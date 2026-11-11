import unittest

import numpy as np

from modules.point_picking import pick_projected


class ProjectedPickingTests(unittest.TestCase):
    def test_nearest_depth_at_same_pixel(self):
        points = np.array([[0., 0., .5], [0., 0., -.5]])
        result = pick_projected(points, np.eye(4), np.eye(4), 100, 100, 50, 50)
        np.testing.assert_array_equal(result, points[1])

    def test_returns_source_coordinate_without_rounding(self):
        points = np.array([[.001, .002, .003]])
        result = pick_projected(points, np.eye(4), np.eye(4), 100, 100, 50, 50)
        np.testing.assert_array_equal(result, points[0])

    def test_empty_or_off_screen_returns_none(self):
        for points in [np.empty((0, 3)), np.array([[10., 10., 0.]]), np.array([[0., 0., 2.]])]:
            self.assertIsNone(pick_projected(points, np.eye(4), np.eye(4), 100, 100, 50, 50))


if __name__ == '__main__':
    unittest.main()
