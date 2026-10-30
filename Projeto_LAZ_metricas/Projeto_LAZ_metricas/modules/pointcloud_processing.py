import numpy as np


def aplicar_amostragem(pontos, fator):
    if fator <= 1:
        return pontos

    pontos_amostrados = pontos[::fator]

    print(f"Total após amostragem: {len(pontos_amostrados):,}")

    return pontos_amostrados


def normalizar_coordenadas(pontos):
    pontos_norm = pontos.copy()

    pontos_norm[:, 0] -= np.min(pontos_norm[:, 0])
    pontos_norm[:, 1] -= np.min(pontos_norm[:, 1])
    pontos_norm[:, 2] -= np.min(pontos_norm[:, 2])

    return pontos_norm


def aplicar_recorte(pontos, recorte):
    mascara = (
        (pontos[:, 0] >= recorte["x_min"]) & (pontos[:, 0] <= recorte["x_max"]) &
        (pontos[:, 1] >= recorte["y_min"]) & (pontos[:, 1] <= recorte["y_max"]) &
        (pontos[:, 2] >= recorte["z_min"]) & (pontos[:, 2] <= recorte["z_max"])
    )

    pontos_recortados = pontos[mascara]

    print(f"Total após recorte: {len(pontos_recortados):,}")

    return pontos_recortados