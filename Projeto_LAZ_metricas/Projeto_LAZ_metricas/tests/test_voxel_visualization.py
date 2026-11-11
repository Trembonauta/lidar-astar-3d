import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from app_3d import Bancada3D
from modules.voxel_visualization import superficie_voxels


class VoxelViewTests(unittest.TestCase):
    def test_single_cube_has_real_size_and_twelve_triangles(self):
        vertices, triangles = superficie_voxels({(2, 3, 4)}, 3.)
        np.testing.assert_array_equal(vertices.min(axis=0), [6, 9, 12])
        np.testing.assert_array_equal(vertices.max(axis=0), [9, 12, 15])
        self.assertEqual(triangles.shape, (12, 3))
        self.assertTrue((triangles < len(vertices)).all())

    def test_adjacent_cubes_omit_only_shared_faces(self):
        vertices, triangles = superficie_voxels({(0, 0, 0), (1, 0, 0)}, 1.)
        self.assertEqual(triangles.shape, (20, 3))
        points = vertices[triangles]
        area = np.linalg.norm(np.cross(points[:, 1]-points[:, 0], points[:, 2]-points[:, 0]), axis=1).sum()/2
        self.assertAlmostEqual(area, 10.)

    def test_empty_and_sparse_large_map(self):
        vertices, triangles = superficie_voxels(set(), 3.)
        self.assertEqual(vertices.shape, (0, 3))
        self.assertEqual(triangles.shape, (0, 3))
        _, triangles = superficie_voxels({(0, 0, 0), (1000000, 1000000, 1000000)}, 1.)
        self.assertEqual(len(triangles), 24)

    def test_toggle_keeps_camera_and_overlays(self):
        viewer = Bancada3D.__new__(Bancada3D)
        viewer.scene = Mock()
        viewer.map_view = SimpleNamespace(selected_index=1)
        viewer.voxel_mesh_ready = True
        viewer.visible_info = SimpleNamespace(text='')
        viewer.preview_count, viewer.detail = 3, np.zeros((2, 3))
        viewer.geometries = {'preview', 'detail', 'voxel_map', 'suavizado', 'collision_boxes'}
        viewer._map_visibility()
        viewer.scene.scene.show_geometry.assert_any_call('voxel_map', True)
        viewer.scene.scene.show_geometry.assert_any_call('detail', False)
        viewer.scene.look_at.assert_not_called()
        viewer.map_view.selected_index = 0
        viewer._map_visibility()
        viewer.scene.scene.show_geometry.assert_any_call('voxel_map', False)
        viewer.scene.scene.show_geometry.assert_any_call('preview', True)
        self.assertIn('Pontos exibidos: 5', viewer.visible_info.text)


if __name__ == '__main__':
    unittest.main()
