"""Seleção por projeção na câmera, retornando um ponto da fonte sem interpolar."""
import numpy as np


def pick_projected(points, projection, view, width, height, x, y, radius=5.):
    transform = np.asarray(projection, dtype=float) @ np.asarray(view, dtype=float)
    best, best_distance, best_depth = None, float('inf'), float('inf')
    for start in range(0, len(points), 200_000):
        block = points[start:start+200_000]
        homogeneous = np.column_stack((block, np.ones(len(block))))
        clip = homogeneous @ transform.T
        front = clip[:, 3] > 0
        ndc = clip[front, :3]/clip[front, 3, None]
        indices = np.flatnonzero(front)
        pixel_x, pixel_y = (ndc[:, 0]+1)*width*.5, (1-ndc[:, 1])*height*.5
        distance = (pixel_x-x)**2+(pixel_y-y)**2
        visible = (distance <= radius**2) & (ndc[:, 2] >= -1) & (ndc[:, 2] <= 1)
        candidates = np.flatnonzero(visible)
        if not len(candidates):
            continue
        # Distância de tela em faixas subpixel; no mesmo pixel prevalece o ponto mais próximo.
        buckets = np.floor(np.sqrt(distance[candidates])*2)
        order = np.lexsort((ndc[candidates, 2], buckets))
        selected = candidates[order[0]]
        score = float(np.floor(np.sqrt(distance[selected])*2))
        depth = float(ndc[selected, 2])
        if (score, depth) < (best_distance, best_depth):
            best = start+int(indices[selected])
            best_distance, best_depth = score, depth
    return None if best is None else np.asarray(points[best], dtype=float).copy()
