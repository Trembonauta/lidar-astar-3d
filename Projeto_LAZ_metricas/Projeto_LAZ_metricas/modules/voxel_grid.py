import numpy as np


def criar_voxel_grid(pontos, voxel_size):
    print("\n===== VOXELIZAÇÃO =====")
    print(f"Tamanho do voxel: {voxel_size} m")

    indices = np.floor(pontos / voxel_size).astype(np.int32)

    indices_unicos = np.unique(indices, axis=0)

    print(f"Pontos originais: {len(pontos):,}")
    print(f"Voxels ocupados: {len(indices_unicos):,}")

    dimensoes = np.max(indices_unicos, axis=0) + 1

    print("Dimensões da grid:")
    print(f"X: {dimensoes[0]}")
    print(f"Y: {dimensoes[1]}")
    print(f"Z: {dimensoes[2]}")
    print(f"Total teórico de voxels: {np.prod(dimensoes):,}")

    occupied_voxels = set(map(tuple, indices_unicos))

    return occupied_voxels, indices_unicos, dimensoes


def voxels_para_pontos(indices_voxels, voxel_size):
    """
    Converte índices de voxels de volta para coordenadas 3D,
    usando o centro de cada voxel.
    """
    pontos_voxels = (indices_voxels + 0.5) * voxel_size
    return pontos_voxels

def coordenada_para_voxel(coordenada, voxel_size):
    """
    Converte coordenada real (x, y, z) para índice de voxel.
    """
    return tuple(np.floor(np.array(coordenada) / voxel_size).astype(np.int32))


def caminho_voxel_para_coordenadas(caminho_voxels, voxel_size):
    """
    Converte caminho em índices de voxel para coordenadas reais,
    usando o centro de cada voxel.
    """
    caminho_voxels = np.array(caminho_voxels)
    return (caminho_voxels + 0.5) * voxel_size

def bloquear_subsolo(indices_voxels, occupied_voxels):
    """
    Para cada coluna (x, y), encontra o voxel ocupado mais baixo
    e marca todos os voxels abaixo dele como ocupados.

    Isso impede o A* de atravessar por baixo do terreno.
    """

    print("\n===== BLOQUEANDO SUBSOLO =====")

    menor_z_por_coluna = {}

    for ix, iy, iz in indices_voxels:
        chave = (ix, iy)

        if chave not in menor_z_por_coluna:
            menor_z_por_coluna[chave] = iz
        else:
            menor_z_por_coluna[chave] = min(menor_z_por_coluna[chave], iz)

    novos_ocupados = set(occupied_voxels)

    for (ix, iy), z_chao in menor_z_por_coluna.items():
        for iz in range(0, z_chao + 1):
            novos_ocupados.add((ix, iy, iz))

    print(f"Voxels ocupados antes: {len(occupied_voxels):,}")
    print(f"Voxels ocupados depois: {len(novos_ocupados):,}")

    return novos_ocupados