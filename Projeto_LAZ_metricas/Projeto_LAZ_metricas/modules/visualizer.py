import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


def visualizar_3d(pontos, nome_janela="Visualizador 3D"):
    pontos = np.asarray(pontos)

    if pontos.ndim != 2 or pontos.shape[1] != 3:
        raise ValueError(
            f"Formato inválido para pontos. Esperado array Nx3, recebido shape {pontos.shape}"
        )

    if not np.issubdtype(pontos.dtype, np.number):
        raise TypeError(
            f"Os pontos precisam ser numéricos. Tipo recebido: {pontos.dtype}"
        )

    print("Abrindo visualização 3D...")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pontos.astype(np.float64))

    z = pontos[:, 2]
    z_norm = (z - z.min()) / (np.ptp(z) + 1e-9)

    cores = plt.cm.terrain(z_norm)[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(cores)

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name=nome_janela,
        width=1280,
        height=720,
        visible=True
    )

    vis.add_geometry(pcd)

    vis.poll_events()
    vis.update_renderer()

    ctr = vis.get_view_control()
    ctr.set_zoom(0.6)

    vis.run()
    vis.destroy_window()
    
def visualizar_3d_com_caminho(pontos, caminho, nome_janela="Mapa 3D com caminho"):
    pontos = np.asarray(pontos)
    caminho = np.asarray(caminho)

    print("Abrindo visualização 3D com caminho...")

    # Nuvem de pontos
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pontos.astype(np.float64))

    z = pontos[:, 2]
    z_norm = (z - z.min()) / (np.ptp(z) + 1e-9)
    cores = plt.cm.terrain(z_norm)[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(cores)

    # Linha do caminho
    linhas = [[i, i + 1] for i in range(len(caminho) - 1)]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(caminho.astype(np.float64))
    line_set.lines = o3d.utility.Vector2iVector(linhas)

    # Vermelho vivo para destacar o caminho
    line_set.colors = o3d.utility.Vector3dVector(
        [[1.0, 0.0, 0.0] for _ in linhas]
    )

    # Esferas para início e fim
    inicio = o3d.geometry.TriangleMesh.create_sphere(radius=10)
    inicio.translate(caminho[0])
    inicio.paint_uniform_color([0.0, 1.0, 0.0])

    fim = o3d.geometry.TriangleMesh.create_sphere(radius=10)
    fim.translate(caminho[-1])
    fim.paint_uniform_color([0.0, 0.0, 1.0])

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name=nome_janela,
        width=1280,
        height=720,
        visible=True
    )

    vis.add_geometry(pcd)
    vis.add_geometry(line_set)
    vis.add_geometry(inicio)
    vis.add_geometry(fim)

    vis.poll_events()
    vis.update_renderer()

    ctr = vis.get_view_control()
    ctr.set_zoom(0.6)

    vis.run()
    vis.destroy_window()
    
def visualizar_3d_com_dois_caminhos(
    pontos,
    caminho_bruto,
    caminho_suavizado,
    nome_janela="Comparação de caminhos"
                                    ):
    pontos = np.asarray(pontos)
    caminho_bruto = np.asarray(caminho_bruto)
    caminho_suavizado = np.asarray(caminho_suavizado)

    print("Abrindo visualização 3D com caminho bruto e suavizado...")

    # Nuvem / voxels
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pontos.astype(np.float64))

    z = pontos[:, 2]
    z_norm = (z - z.min()) / (np.ptp(z) + 1e-9)
    cores = plt.cm.terrain(z_norm)[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(cores)

    # Caminho bruto - vermelho
    linhas_bruto = [[i, i + 1] for i in range(len(caminho_bruto) - 1)]

    line_bruto = o3d.geometry.LineSet()
    line_bruto.points = o3d.utility.Vector3dVector(caminho_bruto.astype(np.float64))
    line_bruto.lines = o3d.utility.Vector2iVector(linhas_bruto)
    line_bruto.colors = o3d.utility.Vector3dVector(
        [[1.0, 0.0, 0.0] for _ in linhas_bruto]
    )

    # Caminho suavizado - azul
    linhas_suave = [[i, i + 1] for i in range(len(caminho_suavizado) - 1)]

    line_suave = o3d.geometry.LineSet()
    line_suave.points = o3d.utility.Vector3dVector(caminho_suavizado.astype(np.float64))
    line_suave.lines = o3d.utility.Vector2iVector(linhas_suave)
    line_suave.colors = o3d.utility.Vector3dVector(
        [[0.0, 0.2, 1.0] for _ in linhas_suave]
    )

    # Início e fim
    inicio = o3d.geometry.TriangleMesh.create_sphere(radius=10)
    inicio.translate(caminho_bruto[0])
    inicio.paint_uniform_color([0.0, 1.0, 0.0])

    fim = o3d.geometry.TriangleMesh.create_sphere(radius=10)
    fim.translate(caminho_bruto[-1])
    fim.paint_uniform_color([1.0, 1.0, 0.0])

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name=nome_janela,
        width=1280,
        height=720,
        visible=True
    )

    vis.add_geometry(pcd)
    vis.add_geometry(line_bruto)
    vis.add_geometry(line_suave)
    vis.add_geometry(inicio)
    vis.add_geometry(fim)

    vis.poll_events()
    vis.update_renderer()

    ctr = vis.get_view_control()
    ctr.set_zoom(0.6)

    vis.run()
    vis.destroy_window()