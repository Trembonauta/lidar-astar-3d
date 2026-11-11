from pathlib import Path
import tempfile
import threading
import unittest
from concurrent.futures import CancelledError

import laspy
import numpy as np

from modules.cloud_index import build_index
from modules.pipeline import preparar_mapa


class IndexTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.source = self.folder/'source.laz'
        header = laspy.LasHeader(point_format=3, version='1.2')
        header.scales = [.001, .001, .001]
        header.offsets = [500000., 7000000., 100.]
        cloud = laspy.LasData(header)
        self.raw = np.array([[0, 0, 0], [1, 2, 3], [49999, 1000, 2000], [50000, 1000, 2000],
                             [99999, 99999, 99999], [100000, 100000, 100000]], dtype=np.int32)
        cloud.X, cloud.Y, cloud.Z = self.raw.T
        cloud.write(self.source)

    def test_full_resolution_and_cache(self):
        index = build_index(self.source, self.folder/'cache', preview_limit=2, chunk_size=2)
        self.assertEqual(index.metadata['count'], 6)
        self.assertLessEqual(len(index.preview), 2)
        region = index.region([50., 50., 50.], 100.)
        actual = sorted(map(tuple, np.rint(region*1000).astype(int)))
        self.assertEqual(actual, sorted(map(tuple, self.raw)))
        cached = build_index(self.source, self.folder/'cache', preview_limit=2)
        self.assertEqual(cached.path, index.path)
        np.testing.assert_array_equal(cached.preview, index.preview)

    def test_radius_filters_exactly_and_preserves_millimeters(self):
        index = build_index(self.source, self.folder/'cache', chunk_size=2)
        region = index.region([0., 0., 0.], .01)
        np.testing.assert_allclose(region, self.raw[:2]*.001, atol=1e-9)
        with self.assertRaises(ValueError):
            index.region([0., 0., 0.], -1.)

    def test_budget_rejects_instead_of_subsampling(self):
        index = build_index(self.source, self.folder/'cache')
        with self.assertRaisesRegex(ValueError, 'densa'):
            index.region([50., 50., 50.], 100., max_points=2)
        region = index.region([50., 50., 50.], 100., max_points=6)
        self.assertEqual(len(region), 6)

    def test_larger_preview_keeps_regional_coordinates(self):
        small = build_index(self.source, self.folder/'cache', preview_limit=2)
        full = build_index(self.source, self.folder/'cache', preview_limit=6)
        self.assertEqual(len(full.preview), 6)
        self.assertNotEqual(small.path, full.path)
        np.testing.assert_array_equal(small.region([50., 50., 50.], 100.),
                                      full.region([50., 50., 50.], 100.))

    def test_cancelled_build_does_not_publish_partial_cache(self):
        event = threading.Event()
        event.set()
        with self.assertRaises(CancelledError):
            build_index(self.source, self.folder/'cache', cancelled=event)
        self.assertEqual(list((self.folder/'cache').iterdir()), [])

    def test_planner_and_viewer_use_same_origin(self):
        index = build_index(self.source, self.folder/'cache')
        mapa = preparar_mapa(self.source, amostragem=1, voxel_size=50., origem_normalizacao=index.metadata['origin'])
        np.testing.assert_allclose(mapa.pontos, self.raw*.001, atol=1e-9)


if __name__ == '__main__':
    unittest.main()
