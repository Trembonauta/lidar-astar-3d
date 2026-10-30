import numpy as np
from scipy.interpolate import splprep, splev


def suavizar_caminho_spline(caminho, num_pontos=1000, suavizacao=50.0):
    """
    Suaviza um caminho 3D usando B-Spline paramétrica.

    caminho: array Nx3 com [x, y, z]
    num_pontos: quantidade de pontos no caminho suavizado
    suavizacao: fator de suavização da spline
    """

    caminho = np.asarray(caminho, dtype=float)

    if caminho.ndim != 2 or caminho.shape[1] != 3:
        raise ValueError("O caminho precisa ter formato Nx3.")

    if len(caminho) < 4:
        raise ValueError("Para spline, o caminho precisa ter pelo menos 4 pontos.")

    # Remove pontos repetidos
    diffs = np.diff(caminho, axis=0)
    distancias = np.linalg.norm(diffs, axis=1)
    mascara = np.insert(distancias > 1e-6, 0, True)
    caminho = caminho[mascara]

    x = caminho[:, 0]
    y = caminho[:, 1]
    z = caminho[:, 2]

    # Parâmetro baseado no comprimento acumulado
    dist_acumulada = np.insert(np.cumsum(np.linalg.norm(np.diff(caminho, axis=0), axis=1)), 0, 0)
    u = dist_acumulada / dist_acumulada[-1]

    # Spline 3D
    tck, _ = splprep([x, y, z], u=u, s=suavizacao)

    u_novo = np.linspace(0, 1, num_pontos)
    x_s, y_s, z_s = splev(u_novo, tck)

    caminho_suavizado = np.vstack((x_s, y_s, z_s)).T

    return caminho_suavizado