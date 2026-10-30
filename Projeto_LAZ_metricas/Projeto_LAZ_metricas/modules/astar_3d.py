import heapq
import time

import numpy as np
from tqdm import tqdm


def heuristica(a, b):
    """Distância Euclidiana 3D."""
    return np.linalg.norm(np.array(a) - np.array(b))


def gerar_vizinhos():
    """Gera os 26 vizinhos possíveis em 3D."""
    vizinhos = []

    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                vizinhos.append((dx, dy, dz))

    return vizinhos


def dentro_dos_limites(no, dimensoes):
    x, y, z = no
    return (
        0 <= x < dimensoes[0]
        and 0 <= y < dimensoes[1]
        and 0 <= z < dimensoes[2]
    )


def reconstruir_caminho(came_from, atual):
    caminho = [atual]

    while atual in came_from:
        atual = came_from[atual]
        caminho.append(atual)

    caminho.reverse()
    return caminho


def astar_3d(
    start,
    goal,
    occupied_voxels,
    dimensoes,
    retornar_estatisticas=False,
    mostrar_progresso=True,
):
    """
    A* 3D usando voxels ocupados como obstáculos.

    Quando retornar_estatisticas=True, retorna (caminho, estatisticas).
    O comportamento original é preservado por padrão.
    """
    print("\n===== A* 3D =====")
    print(f"Start voxel: {start}")
    print(f"Goal voxel: {goal}")

    if not dentro_dos_limites(start, dimensoes):
        raise ValueError("O ponto inicial está fora dos limites da grid.")

    if not dentro_dos_limites(goal, dimensoes):
        raise ValueError("O ponto final está fora dos limites da grid.")

    if start in occupied_voxels:
        raise ValueError("O ponto inicial está dentro de um obstáculo.")

    if goal in occupied_voxels:
        raise ValueError("O ponto final está dentro de um obstáculo.")

    vizinhos = gerar_vizinhos()
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0.0}
    visitados = set()

    iteracoes = 0
    max_fila_aberta = len(open_set)
    inicio_tempo = time.perf_counter()

    barra = None
    if mostrar_progresso:
        barra = tqdm(desc="Executando A* 3D", unit=" nós", dynamic_ncols=True)

    while open_set:
        iteracoes += 1

        if barra is not None:
            barra.update(1)
            if iteracoes % 1000 == 0:
                barra.set_postfix({
                    "Fila": len(open_set),
                    "Visitados": len(visitados),
                })

        _, atual = heapq.heappop(open_set)

        if atual in visitados:
            continue

        visitados.add(atual)

        if atual == goal:
            if barra is not None:
                barra.close()

            tempo = time.perf_counter() - inicio_tempo
            caminho = reconstruir_caminho(came_from, atual)

            estatisticas = {
                "encontrou_caminho": True,
                "tempo_planejamento_s": float(tempo),
                "iteracoes": int(iteracoes),
                "nos_visitados": int(len(visitados)),
                "nos_descobertos": int(len(g_score)),
                "max_fila_aberta": int(max_fila_aberta),
                "numero_voxels_caminho": int(len(caminho)),
                "custo_final_voxels": float(g_score[goal]),
            }

            print("\nCaminho encontrado!")
            print(f"Tempo de planejamento: {tempo:.4f} s")
            print(f"Iterações: {iteracoes:,}")
            print(f"Nós visitados: {len(visitados):,}")
            print(f"Nós descobertos: {len(g_score):,}")
            print(f"Máximo da fila aberta: {max_fila_aberta:,}")
            print(f"Tamanho do caminho: {len(caminho):,} voxels")
            print(f"Custo final: {g_score[goal]:.4f} unidades de voxel")

            if retornar_estatisticas:
                return caminho, estatisticas
            return caminho

        for dx, dy, dz in vizinhos:
            vizinho = (
                atual[0] + dx,
                atual[1] + dy,
                atual[2] + dz,
            )

            if not dentro_dos_limites(vizinho, dimensoes):
                continue

            if vizinho in occupied_voxels:
                continue

            if vizinho in visitados:
                continue

            custo_movimento = np.sqrt(dx**2 + dy**2 + dz**2)
            tentativa_g = g_score[atual] + custo_movimento

            if tentativa_g < g_score.get(vizinho, float("inf")):
                came_from[vizinho] = atual
                g_score[vizinho] = tentativa_g

                f_score = tentativa_g + heuristica(vizinho, goal)
                heapq.heappush(open_set, (f_score, vizinho))
                max_fila_aberta = max(max_fila_aberta, len(open_set))

    if barra is not None:
        barra.close()

    tempo = time.perf_counter() - inicio_tempo
    estatisticas = {
        "encontrou_caminho": False,
        "tempo_planejamento_s": float(tempo),
        "iteracoes": int(iteracoes),
        "nos_visitados": int(len(visitados)),
        "nos_descobertos": int(len(g_score)),
        "max_fila_aberta": int(max_fila_aberta),
        "numero_voxels_caminho": 0,
        "custo_final_voxels": None,
    }

    print("\nNenhum caminho encontrado.")
    print(f"Tempo de planejamento: {tempo:.4f} s")
    print(f"Iterações: {iteracoes:,}")
    print(f"Nós visitados: {len(visitados):,}")

    if retornar_estatisticas:
        return None, estatisticas
    return None


def calcular_distancia_caminho(caminho):
    """Mantida por compatibilidade com o código original."""
    caminho = np.asarray(caminho, dtype=float)
    if len(caminho) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(caminho, axis=0), axis=1).sum())
