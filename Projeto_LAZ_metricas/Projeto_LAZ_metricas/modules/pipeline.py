"""Processamento reutilizável, sem janelas ou dependência da interface."""
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from time import perf_counter

import numpy as np

from modules.astar_3d import astar_3d, dentro_dos_limites
from modules.fuzzy_exporter import salvar_trajetoria_fuzzy_csv
from modules.laz_loader import carregar_laz
from modules.path_smoothing import suavizar_caminho_spline
from modules.path_statistics import (
    calcular_distancia_minima_obstaculos, calcular_metricas_trajetoria,
    salvar_metricas_json, validar_colisao_caminho,
)
from modules.pointcloud_processing import aplicar_amostragem, aplicar_recorte
from modules.voxel_grid import (
    bloquear_subsolo, caminho_voxel_para_coordenadas, coordenada_para_voxel,
    criar_voxel_grid,
)


@dataclass
class Mapa:
    pontos: np.ndarray
    indices: np.ndarray
    ocupados: set
    dimensoes: np.ndarray
    voxel_size: float
    origem: np.ndarray
    tempos: dict
    parametros: dict


def preparar_mapa(arquivo, amostragem=10, voxel_size=5., recorte=None, informar=None, origem_normalizacao=None):
    if isinstance(amostragem, bool) or int(amostragem) != amostragem or amostragem < 1:
        raise ValueError("A amostragem deve ser um inteiro positivo.")
    if not np.isfinite(voxel_size) or voxel_size <= 0:
        raise ValueError("O tamanho do voxel deve ser positivo e finito.")
    informar = informar or (lambda mensagem: None)
    tempos = {}
    informar("Lendo arquivo LiDAR...")
    inicio = perf_counter()
    pontos = carregar_laz(Path(arquivo))
    tempos['leitura_s'] = perf_counter() - inicio
    inicio = perf_counter()
    pontos = aplicar_amostragem(pontos, int(amostragem))
    if not len(pontos) or not np.isfinite(pontos).all():
        raise ValueError("A nuvem está vazia ou contém coordenadas não finitas.")
    origem = (pontos.min(axis=0) if origem_normalizacao is None
              else np.asarray(origem_normalizacao, dtype=float))
    if origem.shape != (3,) or not np.isfinite(origem).all():
        raise ValueError('Origem de normalização inválida.')
    pontos = pontos - origem
    if recorte is not None:
        pontos = aplicar_recorte(pontos, recorte)
    if not len(pontos):
        raise ValueError("O recorte não contém pontos.")
    tempos['processamento_s'] = perf_counter() - inicio
    informar("Voxelizando e bloqueando subsolo...")
    inicio = perf_counter()
    superficie, indices, dimensoes = criar_voxel_grid(pontos, voxel_size)
    ocupados = bloquear_subsolo(indices, superficie)
    tempos['voxelizacao_s'] = perf_counter() - inicio
    parametros = dict(arquivo=str(Path(arquivo).resolve()), amostragem=int(amostragem),
                      voxel_size=float(voxel_size), recorte=recorte)
    return Mapa(pontos, indices, ocupados, dimensoes, float(voxel_size), origem,
                tempos, parametros)


def validar_ponto(mapa, ponto):
    ponto = np.asarray(ponto, dtype=float)
    if ponto.shape != (3,) or not np.isfinite(ponto).all():
        return dict(valido=False, motivo='Coordenadas inválidas.', voxel=None)
    voxel = coordenada_para_voxel(ponto, mapa.voxel_size)
    if not dentro_dos_limites(voxel, mapa.dimensoes):
        motivo = 'Fora dos limites da grade. Reduza a altura ou escolha outro local.'
    elif voxel in mapa.ocupados:
        motivo = 'Voxel ocupado. Ajuste a altura ou escolha outro local.'
    else:
        return dict(valido=True, motivo='Posição livre na grade.', voxel=tuple(map(int, voxel)))
    return dict(valido=False, motivo=motivo, voxel=tuple(map(int, voxel)))


def planejar(mapa, inicio, destino, output_dir=None, informar=None, progresso=False, total_time=800., informar_busca=None):
    informar = informar or (lambda mensagem: None)
    for nome, ponto in [('Início', inicio), ('Destino', destino)]:
        estado = validar_ponto(mapa, ponto)
        if not estado['valido']:
            raise ValueError(f"{nome}: {estado['motivo']}")
    start = coordenada_para_voxel(inicio, mapa.voxel_size)
    goal = coordenada_para_voxel(destino, mapa.voxel_size)
    if start == goal:
        raise ValueError('Início e destino pertencem ao mesmo voxel.')
    informar('Executando A*...')
    caminho, stats = astar_3d(start, goal, mapa.ocupados, mapa.dimensoes,
                             retornar_estatisticas=True, mostrar_progresso=progresso,
                             informar_busca=informar_busca)
    resultado = dict(bruto=None, suavizado=None, metricas={'astar': stats}, arquivos={})
    if caminho is not None:
        informar('Suavizando e avaliando os caminhos...')
        inicio_avaliacao = perf_counter()
        bruto = caminho_voxel_para_coordenadas(caminho, mapa.voxel_size)
        suave = (suavizar_caminho_spline(bruto, num_pontos=1000, suavizacao=80.)
                 if len(bruto) >= 4 else bruto.copy())
        validacao_bruto = validar_colisao_caminho(bruto, mapa.ocupados, mapa.voxel_size)
        validacao_suave = validar_colisao_caminho(suave, mapa.ocupados, mapa.voxel_size)
        metricas = calcular_metricas_trajetoria(
            caminho_bruto=bruto, caminho_suavizado=suave,
            start_real=inicio, goal_real=destino, voxel_size=mapa.voxel_size,
            estatisticas_astar=stats, validacao_bruto=validacao_bruto,
            validacao_suavizado=validacao_suave,
            distancia_minima_bruto=calcular_distancia_minima_obstaculos(bruto, mapa.indices, mapa.voxel_size),
            distancia_minima_suavizado=calcular_distancia_minima_obstaculos(suave, mapa.indices, mapa.voxel_size))
        metricas['tempos'] = {**mapa.tempos, 'avaliacao_s': perf_counter() - inicio_avaliacao}
        resultado.update(bruto=bruto, suavizado=suave, metricas=metricas)
    resultado['metricas']['experimento'] = {
        **mapa.parametros, 'origem_normalizacao': mapa.origem.tolist(),
        'inicio': list(map(float, inicio)), 'destino': list(map(float, destino)),
        'suavizacao': 80., 'pontos_spline': 1000, 'tempo_trajetoria_s': float(total_time),
        'animacao_busca': informar_busca is not None,
    }
    if output_dir is not None:
        pasta = Path(output_dir)
        identificador = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        metricas_path = pasta/'metrics'/f'estatisticas_rota_{identificador}.json'
        salvar_metricas_json(resultado['metricas'], metricas_path)
        resultado['arquivos']['metricas'] = str(metricas_path)
        # Não exportar para seguimento uma spline reprovada pela validação.
        if caminho is not None and resultado['metricas']['caminho_suavizado']['validacao_colisao']['livre_de_colisao']:
            csv_path = pasta/'trajectories'/f'trajetoria_fuzzy_{identificador}.csv'
            salvar_trajetoria_fuzzy_csv(resultado['suavizado'], csv_path, total_time=total_time)
            resultado['arquivos']['trajetoria'] = str(csv_path)
    return resultado
