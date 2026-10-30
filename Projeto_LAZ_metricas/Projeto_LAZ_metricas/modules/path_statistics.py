import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def calcular_comprimento_caminho(caminho):
    """Calcula o comprimento geométrico de uma trajetória 3D."""
    caminho = np.asarray(caminho, dtype=float)

    if caminho.ndim != 2 or caminho.shape[1] != 3:
        raise ValueError("O caminho precisa ter formato Nx3.")

    if len(caminho) < 2:
        return 0.0

    return float(np.linalg.norm(np.diff(caminho, axis=0), axis=1).sum())


def calcular_distancia_direta(inicio, fim):
    """Distância Euclidiana 3D entre origem e destino."""
    inicio = np.asarray(inicio, dtype=float)
    fim = np.asarray(fim, dtype=float)
    return float(np.linalg.norm(fim - inicio))


def calcular_excesso_percentual(comprimento, distancia_direta):
    """Acréscimo percentual do percurso em relação à linha reta."""
    if distancia_direta <= 0:
        return 0.0
    return float((comprimento / distancia_direta - 1.0) * 100.0)


def validar_colisao_caminho(caminho, occupied_voxels, voxel_size, passo=None):
    """
    Verifica segmentos do caminho por amostragem espacial.

    O teste é aplicado ao caminho contínuo, e não apenas aos nós do A*.
    Isso é especialmente importante após a suavização por spline.
    """
    caminho = np.asarray(caminho, dtype=float)

    if passo is None:
        passo = voxel_size / 4.0

    if passo <= 0:
        raise ValueError("O passo de validação deve ser positivo.")

    voxels_em_colisao = set()
    total_amostras = 0

    for i in range(len(caminho) - 1):
        p0 = caminho[i]
        p1 = caminho[i + 1]
        comprimento = np.linalg.norm(p1 - p0)

        n = max(2, int(np.ceil(comprimento / passo)) + 1)
        amostras = np.linspace(p0, p1, n)
        indices = np.floor(amostras / voxel_size).astype(np.int32)

        for indice in indices:
            total_amostras += 1
            voxel = tuple(indice)
            if voxel in occupied_voxels:
                voxels_em_colisao.add(voxel)

    return {
        "livre_de_colisao": len(voxels_em_colisao) == 0,
        "quantidade_voxels_colisao": len(voxels_em_colisao),
        "voxels_colisao": sorted(voxels_em_colisao),
        "passo_validacao_m": float(passo),
        "total_amostras_testadas": int(total_amostras),
    }


def calcular_distancia_minima_obstaculos(caminho, indices_voxels, voxel_size, k=8):
    """
    Estima a menor distância entre a trajetória e os voxels ocupados da
    nuvem de pontos original.

    A distância é calculada até as caixas cúbicas dos voxels, não apenas até
    seus centros. Usa os k centros mais próximos para reduzir custo.
    """
    caminho = np.asarray(caminho, dtype=float)
    indices_voxels = np.asarray(indices_voxels, dtype=float)

    if len(caminho) == 0 or len(indices_voxels) == 0:
        return float("nan")

    centros = (indices_voxels + 0.5) * voxel_size
    arvore = cKDTree(centros)

    k = max(1, min(int(k), len(centros)))
    _, idx = arvore.query(caminho, k=k)

    if k == 1:
        idx = idx[:, None]

    candidatos = centros[idx]
    pontos = caminho[:, None, :]

    # Distância exata de um ponto a uma caixa alinhada aos eixos.
    delta = np.maximum(np.abs(pontos - candidatos) - voxel_size / 2.0, 0.0)
    distancias_caixas = np.linalg.norm(delta, axis=2)

    return float(np.min(distancias_caixas))


def calcular_metricas_trajetoria(
    caminho_bruto,
    caminho_suavizado,
    start_real,
    goal_real,
    voxel_size,
    estatisticas_astar=None,
    validacao_bruto=None,
    validacao_suavizado=None,
    distancia_minima_bruto=None,
    distancia_minima_suavizado=None,
):
    """Consolida as principais métricas úteis para artigo/TCC."""
    bruto = np.asarray(caminho_bruto, dtype=float)
    suave = np.asarray(caminho_suavizado, dtype=float)

    distancia_direta = calcular_distancia_direta(start_real, goal_real)
    comprimento_bruto = calcular_comprimento_caminho(bruto)
    comprimento_suave = calcular_comprimento_caminho(suave)

    metricas = {
        "voxel_size_m": float(voxel_size),
        "start_real_m": [float(v) for v in start_real],
        "goal_real_m": [float(v) for v in goal_real],
        "distancia_direta_m": distancia_direta,
        "caminho_bruto": {
            "numero_pontos": int(len(bruto)),
            "comprimento_m": comprimento_bruto,
            "excesso_sobre_reta_percentual": calcular_excesso_percentual(
                comprimento_bruto, distancia_direta
            ),
            "z_min_m": float(np.min(bruto[:, 2])),
            "z_max_m": float(np.max(bruto[:, 2])),
        },
        "caminho_suavizado": {
            "numero_pontos": int(len(suave)),
            "comprimento_m": comprimento_suave,
            "excesso_sobre_reta_percentual": calcular_excesso_percentual(
                comprimento_suave, distancia_direta
            ),
            "z_min_m": float(np.min(suave[:, 2])),
            "z_max_m": float(np.max(suave[:, 2])),
        },
    }

    if estatisticas_astar is not None:
        metricas["astar"] = estatisticas_astar

    if validacao_bruto is not None:
        metricas["caminho_bruto"]["validacao_colisao"] = validacao_bruto

    if validacao_suavizado is not None:
        metricas["caminho_suavizado"]["validacao_colisao"] = validacao_suavizado

    if distancia_minima_bruto is not None:
        metricas["caminho_bruto"]["distancia_minima_obstaculos_m"] = float(
            distancia_minima_bruto
        )

    if distancia_minima_suavizado is not None:
        metricas["caminho_suavizado"]["distancia_minima_obstaculos_m"] = float(
            distancia_minima_suavizado
        )

    return metricas


def imprimir_metricas(metricas):
    print("\n===== MÉTRICAS DA TRAJETÓRIA =====")
    print(f"Distância direta: {metricas['distancia_direta_m']:.2f} m")

    for chave, nome in [
        ("caminho_bruto", "Caminho A* bruto"),
        ("caminho_suavizado", "Caminho suavizado"),
    ]:
        dados = metricas[chave]
        print(f"\n{nome}:")
        print(f"  Pontos: {dados['numero_pontos']:,}")
        print(f"  Comprimento: {dados['comprimento_m']:.2f} m")
        print(
            "  Acréscimo sobre a linha reta: "
            f"{dados['excesso_sobre_reta_percentual']:.2f}%"
        )
        print(f"  Faixa de altitude relativa: {dados['z_min_m']:.2f} a {dados['z_max_m']:.2f} m")

        if "distancia_minima_obstaculos_m" in dados:
            print(
                "  Distância mínima aos voxels de obstáculo: "
                f"{dados['distancia_minima_obstaculos_m']:.2f} m"
            )

        validacao = dados.get("validacao_colisao")
        if validacao is not None:
            status = "SIM" if validacao["livre_de_colisao"] else "NÃO"
            print(f"  Livre de colisão (validação amostrada): {status}")

    if "astar" in metricas:
        astar = metricas["astar"]
        print("\nA* 3D:")
        print(f"  Tempo de planejamento: {astar['tempo_planejamento_s']:.4f} s")
        print(f"  Iterações: {astar['iteracoes']:,}")
        print(f"  Nós visitados: {astar['nos_visitados']:,}")
        print(f"  Nós descobertos: {astar['nos_descobertos']:,}")
        print(f"  Máximo da fila aberta: {astar['max_fila_aberta']:,}")
        if astar.get("custo_final_voxels") is not None:
            print(f"  Custo final em unidades de voxel: {astar['custo_final_voxels']:.4f}")


def salvar_metricas_json(metricas, caminho_arquivo):
    """
    Salva as métricas em formato JSON, convertendo tipos NumPy
    para tipos nativos do Python.
    """

    caminho_arquivo = Path(caminho_arquivo)
    caminho_arquivo.parent.mkdir(parents=True, exist_ok=True)

    def converter_para_python(obj):
        # Dicionários
        if isinstance(obj, dict):
            return {
                str(chave): converter_para_python(valor)
                for chave, valor in obj.items()
            }

        # Listas, tuplas e conjuntos
        if isinstance(obj, (list, tuple, set)):
            return [
                converter_para_python(valor)
                for valor in obj
            ]

        # Arrays NumPy
        if isinstance(obj, np.ndarray):
            return converter_para_python(obj.tolist())

        # Inteiros NumPy: int32, int64 etc.
        if isinstance(obj, np.integer):
            return int(obj)

        # Floats NumPy: float32, float64 etc.
        if isinstance(obj, np.floating):
            return float(obj)

        # Booleanos NumPy
        if isinstance(obj, np.bool_):
            return bool(obj)

        # Path
        if isinstance(obj, Path):
            return str(obj)

        # Tipos Python normais
        return obj

    metricas_convertidas = converter_para_python(metricas)

    with caminho_arquivo.open("w", encoding="utf-8") as arquivo:
        json.dump(
            metricas_convertidas,
            arquivo,
            indent=2,
            ensure_ascii=False
        )

    print(f"\nEstatísticas salvas em: {caminho_arquivo}")
