import laspy
import numpy as np


def carregar_laz(caminho_arquivo):
    print("Lendo arquivo LAZ...")

    las = laspy.read(caminho_arquivo)

    x = np.array(las.x)
    y = np.array(las.y)
    z = np.array(las.z)

    pontos = np.vstack((x, y, z)).T

    print(f"Total de pontos original: {len(pontos):,}")

    return pontos
