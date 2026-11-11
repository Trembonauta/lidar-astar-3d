"""Execução tradicional com as mesmas etapas utilizadas pela bancada."""
import config
from modules.pipeline import preparar_mapa, planejar
from modules.map_statistics import calcular_estatisticas
from modules.path_statistics import imprimir_metricas
from modules.voxel_grid import voxels_para_pontos


def main():
    # A visualização fica neste ponto de entrada; pipeline.py não abre janelas.
    from modules.visualizer import (
        visualizar_3d, visualizar_3d_com_caminho, visualizar_3d_com_dois_caminhos,
    )

    mapa = preparar_mapa(
        config.LAZ_PATH, config.AMOSTRAGEM, config.VOXEL_SIZE,
        config.RECORTE if config.USAR_RECORTE else None,
        informar=print,
        origem_normalizacao=config.ORIGEM_NORMALIZACAO,
    )
    print(calcular_estatisticas(mapa.pontos))
    exibicao = (voxels_para_pontos(mapa.indices, mapa.voxel_size)
                if config.VISUALIZAR_VOXELS else mapa.pontos)
    visualizar_3d(exibicao, nome_janela='Mapa preparado')
    resultado = planejar(mapa, config.START_REAL, config.GOAL_REAL,
                         output_dir=config.OUTPUT_DIR, informar=print, progresso=True,
                         total_time=config.FUZZY_TOTAL_TIME)
    if resultado['bruto'] is None:
        return
    imprimir_metricas(resultado['metricas'])
    if 'trajetoria' not in resultado['arquivos']:
        print('CSV não exportado: a trajetória suavizada apresentou colisão.')
    visualizar_3d_com_dois_caminhos(exibicao, resultado['bruto'], resultado['suavizado'])
    visualizar_3d_com_caminho(exibicao, resultado['bruto'])


if __name__ == '__main__':
    main()
