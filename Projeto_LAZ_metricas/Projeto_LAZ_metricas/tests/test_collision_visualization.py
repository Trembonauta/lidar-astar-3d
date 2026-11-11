import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from app_3d import Bancada3D
from modules.collision_visualization import caixas_colisao
from modules.path_statistics import validar_colisao_caminho


class CollisionDiagnosticsTests(unittest.TestCase):
    def test_reports_interior_collision_with_free_endpoints(self):
        result = validar_colisao_caminho([[.1, .5, .5], [2.9, .5, .5]], {(1, 0, 0)}, 1.)
        self.assertFalse(result['livre_de_colisao'])
        self.assertEqual(result['voxels_colisao'], [(1, 0, 0)])
        self.assertEqual(result['segmentos_colisao'], [0])
        points = np.asarray(result['pontos_colisao_m'])
        self.assertTrue(len(points) > 0)
        np.testing.assert_array_equal(np.floor(points), np.tile([1, 0, 0], (len(points), 1)))
        self.assertEqual(result['total_amostras_testadas'], 13)

    def test_free_path_has_no_markers(self):
        result = validar_colisao_caminho([[0, 0, 0], [1, 0, 0]], set(), 1.)
        self.assertTrue(result['livre_de_colisao'])
        self.assertEqual(result['pontos_colisao_m'], [])
        self.assertEqual(result['segmentos_colisao'], [])
        vertices, edges = caixas_colisao([], 5.)
        self.assertEqual(vertices.shape, (0, 3))
        self.assertEqual(edges.shape, (0, 2))

    def test_boxes_use_voxel_corner_not_center_as_origin(self):
        vertices, edges = caixas_colisao([[2, 3, 4], [-1, 0, 1]], 5.)
        np.testing.assert_array_equal(vertices[:8].min(axis=0), [10, 15, 20])
        np.testing.assert_array_equal(vertices[:8].max(axis=0), [15, 20, 25])
        self.assertEqual(edges.shape, (24, 2))
        self.assertTrue((edges[12:] >= 8).all())
        self.assertTrue((edges < len(vertices)).all())

    def test_focus_cycles_and_uses_result_voxel_size(self):
        viewer = Bancada3D.__new__(Bancada3D)
        viewer.result = {'metricas': {'voxel_size_m': 5., 'caminho_suavizado': {
            'validacao_colisao': {'voxels_colisao': [[2, 3, 4], [5, 6, 7]]}}}}
        viewer.index = SimpleNamespace(metadata={'origin': [500000., 7000000., 200.]})
        viewer.busy = False
        viewer.collision_index = -1
        viewer.keys = {}
        viewer.scene = Mock()
        viewer.window = Mock()
        viewer.collision_info = Mock()
        viewer._region = Mock()
        viewer._focus_collision(1)
        np.testing.assert_array_equal(viewer.focus, [12.5, 17.5, 22.5])
        self.assertIn('500012.500', viewer.collision_info.text)
        viewer._focus_collision(1)
        self.assertEqual(viewer.collision_index, 1)
        viewer._focus_collision(1)
        self.assertEqual(viewer.collision_index, 0)
        viewer._focus_collision(-1)
        self.assertEqual(viewer.collision_index, 1)


if __name__ == '__main__':
    unittest.main()
