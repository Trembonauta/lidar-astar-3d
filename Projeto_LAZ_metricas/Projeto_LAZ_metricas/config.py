from datetime import datetime
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
LAZ_PATH = BASE_DIR / "data" / "points.laz"

AMOSTRAGEM = 10

USAR_RECORTE = False

RECORTE = {
    "x_min": 0,
    "x_max": 300,
    "y_min": 0,
    "y_max": 300,
    "z_min": -50,
    "z_max": 150,
}

# Voxelização
VOXEL_SIZE = 5.0
VISUALIZAR_VOXELS = True

# Configuração da inflação
INFLAR_OBSTACULOS = False
MARGEM_SEGURANCA = 0.0

# Início e fim do A*
START_REAL = (100.0, 100.0, 120.0)
GOAL_REAL = (3200.0, 2000.0, 250.0)

# Configuração do FUZZY
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

FUZZY_CSV_PATH = (
    f"outputs/trajetoria_fuzzy_{timestamp}.csv"
)
FUZZY_TOTAL_TIME = 800.0

# Estatísticas da rota
PATH_STATS_PATH = f"outputs/estatisticas_rota_{timestamp}.json"