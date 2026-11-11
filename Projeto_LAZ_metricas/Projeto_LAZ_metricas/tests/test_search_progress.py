import unittest
from itertools import count
from unittest.mock import Mock, patch

import numpy as np

from modules.astar_3d import astar_3d
from app_3d import Bancada3D


class SearchProgressTests(unittest.TestCase):
    def test_animation_preserves_path_cost_and_node_count(self):
        args = ((0, 0, 0), (9, 8, 2), {(4, 4, 1)}, (12, 12, 5))
        reference, stats = astar_3d(*args, retornar_estatisticas=True, mostrar_progresso=False)
        frames = []
        actual, animated = astar_3d(*args, retornar_estatisticas=True,
                                    mostrar_progresso=False, informar_busca=frames.append)
        self.assertEqual(actual, reference)
        self.assertEqual(animated['custo_final_voxels'], stats['custo_final_voxels'])
        self.assertEqual(animated['nos_visitados'], stats['nos_visitados'])
        self.assertTrue(frames[-1]['final'])
        self.assertEqual(frames[-1]['atual'], args[1])
        self.assertEqual(frames[-1]['visitados'], animated['nos_visitados'])
        self.assertEqual(frames[-1]['caminho'].tolist(), [list(p) for p in actual])
        for frame in frames:
            path = frame['caminho']
            self.assertEqual(tuple(path[0]), args[0])
            self.assertEqual(tuple(path[-1]), frame['atual'])
            for left, right in zip(path, path[1:]):
                self.assertLessEqual(max(abs(right-left)), 1)
                self.assertNotIn(tuple(right), args[2])

    def test_no_path_also_emits_final_frame(self):
        frames = []
        result = astar_3d((0, 0, 0), (2, 0, 0), {(1, 0, 0)}, (3, 1, 1),
                          mostrar_progresso=False, informar_busca=frames.append)
        self.assertIsNone(result)
        self.assertTrue(frames[-1]['final'])
        self.assertEqual(frames[-1]['visitados'], 1)

    def test_intermediate_frames_contain_connected_paths(self):
        frames = []
        clock = count(0., .2)
        occupied = {(10, y, 0) for y in range(19)}
        with patch('modules.astar_3d.time.perf_counter', side_effect=lambda: next(clock)):
            astar_3d((0, 2, 0), (20, 2, 0), occupied, (25, 25, 1),
                      mostrar_progresso=False, informar_busca=frames.append)
        self.assertGreater(len(frames), 1)
        for frame in frames:
            path = frame['caminho']
            self.assertEqual(tuple(path[0]), (0, 2, 0))
            self.assertEqual(tuple(path[-1]), frame['atual'])
            self.assertTrue((np.abs(np.diff(path, axis=0)) <= 1).all())

    def test_animation_draws_orange_path_without_moving_camera(self):
        viewer = Bancada3D.__new__(Bancada3D)
        viewer.search_voxel = 3.
        viewer.scene, viewer.window = Mock(), Mock()
        viewer.geometries = set()
        viewer._remove = Mock()
        viewer._notice = Mock()
        viewer._search_frame({'caminho': [[0, 0, 0], [1, 0, 0]], 'atual': (1, 0, 0),
                              'visitados': 2, 'fila': 3, 'tempo_s': .2})
        self.assertIn('search_path', viewer.geometries)
        self.assertNotIn('search_visited', viewer.geometries)
        viewer.scene.look_at.assert_not_called()
        geometry = viewer.scene.scene.add_geometry.call_args_list[0].args[1]
        np.testing.assert_allclose(np.asarray(geometry.points), [[1.5, 1.5, 1.5], [4.5, 1.5, 1.5]])
        np.testing.assert_allclose(np.asarray(geometry.colors), [[1., .45, 0.]])


if __name__ == '__main__':
    unittest.main()
