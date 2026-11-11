from datetime import datetime
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
LAZ_PATH = BASE_DIR / "data" / "reboucas_alves_guimaraes_2017.laz"
TEST_MAP_PATH = LAZ_PATH
# Origem do mapa unificado e posições da configuração inicial de testes.
ORIGEM_NORMALIZACAO = (328824.86, 7393205.83, 748.08)
TEST_START_ORIGINAL = (328995.300, 7393497.750, 777.670)
TEST_GOAL_ORIGINAL = (329767.120, 7393756.160, 808.140)
TEST_HEIGHT = 5.0
OUTPUT_DIR = BASE_DIR / "outputs"
IMAGES_DIR = OUTPUT_DIR / "images"

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
VOXEL_SIZE = 3.0
VISUALIZAR_VOXELS = True

# Início e fim do A*
START_REAL = (170.440, 291.920, 29.590)
GOAL_REAL = (942.260, 550.330, 60.060)

# Configuração do FUZZY
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

FUZZY_CSV_PATH = OUTPUT_DIR / "trajectories" / f"trajetoria_fuzzy_{timestamp}.csv"
FUZZY_TOTAL_TIME = 800.0

# Estatísticas da rota
PATH_STATS_PATH = OUTPUT_DIR / "metrics" / f"estatisticas_rota_{timestamp}.json"
