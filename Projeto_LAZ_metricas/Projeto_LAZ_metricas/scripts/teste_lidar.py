from pathlib import Path

import laspy
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


# =========================
# CONFIGURAÇÕES
# =========================

arquivo_laz = Path(__file__).resolve().parents[1] / "data" / "points.laz"

# Use valores maiores se estiver pesado
amostragem = 10  # pega 1 ponto a cada 10

# Recorte opcional em metros depois da normalização
usar_recorte = False
x_min, x_max = 0, 300
y_min, y_max = 0, 300
z_min, z_max = -50, 150


# =========================
# 1. LER ARQUIVO LAZ
# =========================

print("Lendo arquivo LAZ...")
las = laspy.read(arquivo_laz)

x = las.x
y = las.y
z = las.z

print(f"Total de pontos original: {len(x):,}")


# =========================
# 2. AMOSTRAGEM
# =========================

x = x[::amostragem]
y = y[::amostragem]
z = z[::amostragem]

print(f"Total após amostragem: {len(x):,}")


# =========================
# 3. NORMALIZAR COORDENADAS
# =========================
# Isso evita trabalhar com coordenadas UTM gigantes

x = x - np.min(x)
y = y - np.min(y)
z = z - np.min(z)

pontos = np.vstack((x, y, z)).T


# =========================
# 4. RECORTE OPCIONAL
# =========================

if usar_recorte:
    mascara = (
        (pontos[:, 0] >= x_min) & (pontos[:, 0] <= x_max) &
        (pontos[:, 1] >= y_min) & (pontos[:, 1] <= y_max) &
        (pontos[:, 2] >= z_min) & (pontos[:, 2] <= z_max)
    )

    pontos = pontos[mascara]

print(f"Total final de pontos: {len(pontos):,}")


# =========================
# 5. PLOT 2D ALTITUDE
# =========================

plt.figure(figsize=(10, 8))
plt.scatter(
    pontos[:, 0],
    pontos[:, 1],
    c=pontos[:, 2],
    s=0.2,
    cmap="terrain"
)
plt.colorbar(label="Altura relativa (m)")
plt.xlabel("X relativo (m)")
plt.ylabel("Y relativo (m)")
plt.title("Mapa 2D da nuvem de pontos")
plt.axis("equal")
plt.show()


# =========================
# 6. PLOT 3D COM OPEN3D
# =========================

print("Abrindo visualização 3D...")

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pontos)

# Colorir por altura
z_norm = (pontos[:, 2] - pontos[:, 2].min()) / (np.ptp(pontos[:, 2]) + 1e-9)
cores = plt.cm.terrain(z_norm)[:, :3]
pcd.colors = o3d.utility.Vector3dVector(cores)

o3d.visualization.draw_geometries([pcd])
