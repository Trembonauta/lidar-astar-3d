from config import (
    LAZ_PATH,
    AMOSTRAGEM,
    USAR_RECORTE,
    RECORTE,
    VOXEL_SIZE,
    VISUALIZAR_VOXELS,
    START_REAL,
    GOAL_REAL,
    FUZZY_CSV_PATH,
    FUZZY_TOTAL_TIME,
    PATH_STATS_PATH,
)

from modules.laz_loader import carregar_laz
from modules.pointcloud_processing import (
    aplicar_amostragem,
    normalizar_coordenadas,
    aplicar_recorte,
)
from modules.map_statistics import calcular_estatisticas
from modules.voxel_grid import (
    criar_voxel_grid,
    voxels_para_pontos,
    coordenada_para_voxel,
    caminho_voxel_para_coordenadas,
    bloquear_subsolo,
)
from modules.astar_3d import astar_3d
from modules.visualizer import (
    visualizar_3d,
    visualizar_3d_com_caminho,
    visualizar_3d_com_dois_caminhos,
)
from modules.path_smoothing import suavizar_caminho_spline
from modules.fuzzy_exporter import salvar_trajetoria_fuzzy_csv
from modules.path_statistics import (
    validar_colisao_caminho,
    calcular_distancia_minima_obstaculos,
    calcular_metricas_trajetoria,
    imprimir_metricas,
    salvar_metricas_json,
)


def main():
    pontos = carregar_laz(LAZ_PATH)
    pontos = aplicar_amostragem(pontos, AMOSTRAGEM)
    pontos = normalizar_coordenadas(pontos)

    if USAR_RECORTE:
        pontos = aplicar_recorte(pontos, RECORTE)

    estatisticas = calcular_estatisticas(pontos)
    print(estatisticas)
    print(f"Total final de pontos: {len(pontos):,}")

    # A voxelização é executada apenas uma vez.
    occupied_voxels_superficie, indices_voxels, dimensoes = criar_voxel_grid(
        pontos, VOXEL_SIZE
    )

    if VISUALIZAR_VOXELS:
        pontos_voxels = voxels_para_pontos(indices_voxels, VOXEL_SIZE)
        visualizar_3d(pontos_voxels, nome_janela="Mapa Voxelizado")
    else:
        visualizar_3d(pontos, nome_janela="Nuvem Original")

    occupied_voxels = bloquear_subsolo(
        indices_voxels,
        occupied_voxels_superficie,
    )

    start_voxel = coordenada_para_voxel(START_REAL, VOXEL_SIZE)
    goal_voxel = coordenada_para_voxel(GOAL_REAL, VOXEL_SIZE)

    caminho_voxels, estatisticas_astar = astar_3d(
        start=start_voxel,
        goal=goal_voxel,
        occupied_voxels=occupied_voxels,
        dimensoes=dimensoes,
        retornar_estatisticas=True,
        mostrar_progresso=True,
    )

    if caminho_voxels is None:
        salvar_metricas_json({"astar": estatisticas_astar}, PATH_STATS_PATH)
        return

    caminho_real = caminho_voxel_para_coordenadas(
        caminho_voxels,
        VOXEL_SIZE,
    )

    caminho_suavizado = suavizar_caminho_spline(
        caminho_real,
        num_pontos=1000,
        suavizacao=80.0,
    )

    print("\n===== SUAVIZAÇÃO DO CAMINHO =====")
    print(f"Pontos do caminho bruto: {len(caminho_real)}")
    print(f"Pontos do caminho suavizado: {len(caminho_suavizado)}")

    # Validação contínua aproximada da trajetória, inclusive após a spline.
    validacao_bruto = validar_colisao_caminho(
        caminho_real,
        occupied_voxels,
        VOXEL_SIZE,
    )
    validacao_suavizado = validar_colisao_caminho(
        caminho_suavizado,
        occupied_voxels,
        VOXEL_SIZE,
    )

    # Clearance em relação aos voxels ocupados originados da nuvem LiDAR.
    distancia_minima_bruto = calcular_distancia_minima_obstaculos(
        caminho_real,
        indices_voxels,
        VOXEL_SIZE,
    )
    distancia_minima_suavizado = calcular_distancia_minima_obstaculos(
        caminho_suavizado,
        indices_voxels,
        VOXEL_SIZE,
    )

    metricas = calcular_metricas_trajetoria(
        caminho_bruto=caminho_real,
        caminho_suavizado=caminho_suavizado,
        start_real=START_REAL,
        goal_real=GOAL_REAL,
        voxel_size=VOXEL_SIZE,
        estatisticas_astar=estatisticas_astar,
        validacao_bruto=validacao_bruto,
        validacao_suavizado=validacao_suavizado,
        distancia_minima_bruto=distancia_minima_bruto,
        distancia_minima_suavizado=distancia_minima_suavizado,
    )

    imprimir_metricas(metricas)
    salvar_metricas_json(metricas, PATH_STATS_PATH)

    pontos_voxels = voxels_para_pontos(indices_voxels, VOXEL_SIZE)
    visualizar_3d_com_dois_caminhos(
        pontos_voxels,
        caminho_real,
        caminho_suavizado,
        nome_janela="A* bruto x Caminho suavizado",
    )

    if VISUALIZAR_VOXELS:
        visualizar_3d_com_caminho(
            pontos_voxels,
            caminho_real,
            nome_janela="Mapa Voxelizado com caminho A*",
        )
    else:
        visualizar_3d_com_caminho(
            pontos,
            caminho_real,
            nome_janela="Nuvem Original com caminho A*",
        )

    salvar_trajetoria_fuzzy_csv(
        caminho_suavizado,
        FUZZY_CSV_PATH,
        total_time=FUZZY_TOTAL_TIME,
    )


if __name__ == "__main__":
    main()
