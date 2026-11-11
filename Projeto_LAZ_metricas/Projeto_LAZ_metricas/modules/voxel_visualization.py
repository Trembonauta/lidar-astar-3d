"""Superfície externa da ocupação: faces internas não precisam ser desenhadas."""
import numpy as np


def superficie_voxels(ocupados, tamanho):
    cells = np.asarray(list(ocupados), dtype=np.int32).reshape(-1, 3)
    if not len(cells):
        return np.empty((0, 3)), np.empty((0, 3), dtype=np.int32)
    keys = np.ascontiguousarray(cells).view(np.dtype((np.void, 12))).ravel()
    faces = [((1, 0, 0), [[1,0,0],[1,1,0],[1,1,1],[1,0,1]]),
             ((-1, 0, 0), [[0,0,0],[0,0,1],[0,1,1],[0,1,0]]),
             ((0, 1, 0), [[0,1,0],[0,1,1],[1,1,1],[1,1,0]]),
             ((0, -1, 0), [[0,0,0],[1,0,0],[1,0,1],[0,0,1]]),
             ((0, 0, 1), [[0,0,1],[1,0,1],[1,1,1],[0,1,1]]),
             ((0, 0, -1), [[0,0,0],[0,1,0],[1,1,0],[1,0,0]])]
    vertices = []
    for direction, corners in faces:
        neighbors = np.ascontiguousarray(cells+np.asarray(direction, dtype=np.int32))
        hidden = np.isin(neighbors.view(np.dtype((np.void, 12))).ravel(), keys)
        vertices.append(((cells[~hidden, None, :]+np.asarray(corners))*tamanho).reshape(-1, 3))
    vertices = np.concatenate(vertices)
    offsets = np.arange(len(vertices)//4, dtype=np.int32)*4
    triangles = (offsets[:, None, None]+np.array([[0,1,2],[0,2,3]], dtype=np.int32)).reshape(-1, 3)
    return vertices, triangles
