"""Geometria de diagnóstico nas mesmas coordenadas locais da grade do A*."""
import numpy as np


def caixas_colisao(voxels, voxel_size):
    """Retorna vértices e arestas das caixas ocupadas, sem alterar a validação."""
    voxels = np.asarray(voxels, dtype=float).reshape(-1, 3)
    corners = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
                        [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]])
    edges = np.array([[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6],
                      [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]])
    vertices = ((voxels[:, None, :] + corners) * voxel_size).reshape(-1, 3)
    lines = (edges[None, :, :] + np.arange(len(voxels))[:, None, None]*8).reshape(-1, 2)
    return vertices, lines
