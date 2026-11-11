import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from scipy.spatial import cKDTree

from app_3d import Bancada3D
import config


class NativeSelectionTests(unittest.TestCase):
    def setUp(self):
        # Sem janela gráfica: testar especificamente a correspondência com a fonte.
        self.viewer = Bancada3D.__new__(Bancada3D)
        self.viewer.mode = 'inicio'
        self.viewer.bases = {}
        self.viewer.detail = np.array([[1.001, 2.002, 3.003], [4., 5., 6.]])
        self.viewer.detail_tree = cKDTree(self.viewer.detail)
        self.viewer._notice = Mock()
        self.viewer._height_changed = Mock()

    def test_pick_uses_original_coordinate_not_depth_estimate(self):
        self.viewer._pick([1.01, 2.01, 3.01])
        np.testing.assert_array_equal(self.viewer.bases['inicio'], self.viewer.detail[0])
        self.viewer._height_changed.assert_called_once_with('inicio')

    def test_distant_overview_cannot_define_endpoint(self):
        self.viewer._pick([20., 20., 20.])
        self.assertFalse(self.viewer.bases)
        self.viewer._height_changed.assert_not_called()

    def test_requires_loaded_original_points(self):
        self.viewer.detail_tree = None
        self.viewer._pick([1., 2., 3.])
        self.assertFalse(self.viewer.bases)

    def test_inspection_loads_region_instead_of_defining_endpoint(self):
        self.viewer.mode = 'inspect'
        self.viewer._region = Mock()
        self.viewer._pick([1., 2., 3.])
        self.assertFalse(self.viewer.bases)
        np.testing.assert_array_equal(self.viewer.focus, [1., 2., 3.])
        self.viewer._region.assert_called_once()

    def test_restores_test_points_without_adding_height_twice(self):
        viewer = self.viewer
        viewer.busy, viewer.prepared, viewer.job = False, False, None
        viewer.index = SimpleNamespace(metadata={'source': str(config.TEST_MAP_PATH),
                                                 'origin': config.ORIGEM_NORMALIZACAO})
        viewer.positions, viewer.valid = {}, {}
        viewer.heights = {name: SimpleNamespace(double_value=0.) for name in ('inicio', 'destino')}
        for method in ('_clear_paths', '_markers', '_position_texts', '_buttons'):
            setattr(viewer, method, Mock())
        viewer._restore_test_points()
        np.testing.assert_allclose(viewer.positions['inicio'], config.START_REAL, atol=1e-8)
        np.testing.assert_allclose(viewer.positions['destino'], config.GOAL_REAL, atol=1e-8)
        np.testing.assert_allclose(viewer.bases['inicio']+[0, 0, 5.], config.START_REAL)
        viewer.positions.clear()
        viewer.index.metadata['source'] = str(config.BASE_DIR/'data'/'outro.laz')
        viewer._restore_test_points()
        self.assertFalse(viewer.positions)


if __name__ == '__main__':
    unittest.main()
