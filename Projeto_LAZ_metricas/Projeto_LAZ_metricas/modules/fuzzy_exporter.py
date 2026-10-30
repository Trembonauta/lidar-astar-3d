from pathlib import Path

import numpy as np
import pandas as pd


def gerar_trajetoria_fuzzy(caminho, total_time=800.0):
    caminho = np.asarray(caminho, dtype=float)

    if caminho.ndim != 2 or caminho.shape[1] != 3:
        raise ValueError("O caminho precisa ter formato Nx3.")

    n = len(caminho)

    t = np.linspace(0.0, total_time, n)

    x = caminho[:, 0]
    y = caminho[:, 1]
    z = caminho[:, 2]

    x_dot = np.gradient(x, t)
    y_dot = np.gradient(y, t)
    z_dot = np.gradient(z, t)

    x_dot_dot = np.gradient(x_dot, t)
    y_dot_dot = np.gradient(y_dot, t)
    z_dot_dot = np.gradient(z_dot, t)

    df = pd.DataFrame({
        "t": t,
        "x": x,
        "y": y,
        "z": z,
        "x_dot": x_dot,
        "y_dot": y_dot,
        "z_dot": z_dot,
        "x_dot_dot": x_dot_dot,
        "y_dot_dot": y_dot_dot,
        "z_dot_dot": z_dot_dot,
    })

    return df


def salvar_trajetoria_fuzzy_csv(caminho, caminho_csv, total_time=800.0):
    df = gerar_trajetoria_fuzzy(
        caminho,
        total_time=total_time
    )

    caminho_csv = Path(caminho_csv)
    caminho_csv.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(caminho_csv, index=False)

    print("\n===== CSV FUZZY GERADO =====")
    print(f"Arquivo salvo em: {caminho_csv}")
    print(f"Amostras: {len(df)}")
    print(f"Tempo total: {total_time:.2f} s")

    return df

