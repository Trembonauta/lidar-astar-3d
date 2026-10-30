import numpy as np


def calcular_estatisticas(pontos):

    x_min = np.min(pontos[:, 0])
    x_max = np.max(pontos[:, 0])

    y_min = np.min(pontos[:, 1])
    y_max = np.max(pontos[:, 1])

    z_min = np.min(pontos[:, 2])
    z_max = np.max(pontos[:, 2])

    tamanho_x = x_max - x_min
    tamanho_y = y_max - y_min
    tamanho_z = z_max - z_min

    print("\n===== ESTATÍSTICAS DO MAPA =====")

    print(f"Tamanho X: {tamanho_x:.2f} m")
    print(f"Tamanho Y: {tamanho_y:.2f} m")
    print(f"Tamanho Z: {tamanho_z:.2f} m")

    print(f"\nX mínimo: {x_min:.2f}")
    print(f"X máximo: {x_max:.2f}")

    print(f"\nY mínimo: {y_min:.2f}")
    print(f"Y máximo: {y_max:.2f}")

    print(f"\nZ mínimo: {z_min:.2f}")
    print(f"Z máximo: {z_max:.2f}")

    print(f"\nTotal de pontos: {len(pontos):,}")

    return {
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "z_min": z_min,
        "z_max": z_max,
        "tamanho_x": tamanho_x,
        "tamanho_y": tamanho_y,
        "tamanho_z": tamanho_z,
    }

