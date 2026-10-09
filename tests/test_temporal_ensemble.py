from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "client"))
from temporal_ensemble import TemporalEnsemble


class TemporalEnsembleTests(unittest.TestCase):
    def test_time_alignment_and_uncovered_tail(self):
        ensemble = TemporalEnsemble(decay=0)
        old = np.repeat(np.arange(4.0)[:, None], 7, axis=1)
        ensemble.fuse(old, 10.0, 0.1)
        fused, info = ensemble.fuse(np.full((4, 7), 10.0), 10.15, 0.1)
        np.testing.assert_allclose(fused[:, 0], [5.75, 6.25, 10, 10])
        self.assertEqual(info["contributors_per_step"], [2, 2, 1, 1])

    def test_oldest_first_exponential_and_zero_is_valid(self):
        ensemble = TemporalEnsemble(decay=0.7)
        ensemble.fuse(np.zeros((4, 7)), 0, 0.1)
        fused, _ = ensemble.fuse(np.ones((3, 7)), 0.1, 0.1)
        np.testing.assert_allclose(fused, np.exp(-0.7) / (1 + np.exp(-0.7)))

    def test_no_recursive_fusion_and_gripper_bypass(self):
        ensemble = TemporalEnsemble(decay=0, include_gripper=False)
        ensemble.fuse(np.zeros((6, 7)), 0, 0.1)
        ensemble.fuse(np.full((5, 7), 6), 0.1, 0.1)
        fused, _ = ensemble.fuse(np.full((4, 7), 12), 0.2, 0.1)
        np.testing.assert_allclose(fused[:, :6], 6)
        np.testing.assert_allclose(fused[:, 6], 12)

    def test_expiry_reset_period_and_validation(self):
        ensemble = TemporalEnsemble()
        raw = np.ones((3, 7))
        for timestamp, period in ((0, 0.1), (1, 0.1), (0, 0.1), (0.1, 0.2)):
            fused, info = ensemble.fuse(raw, timestamp, period)
            np.testing.assert_allclose(fused, raw)
            self.assertEqual(info["history_chunks"], 1)
        ensemble.clear()
        self.assertEqual(ensemble.fuse(raw, 0.2, 0.2)[1]["history_chunks"], 1)
        with self.assertRaises(ValueError):
            TemporalEnsemble(decay=float("nan"))
        with self.assertRaises(ValueError):
            ensemble.fuse(np.full((2, 7), np.nan), 0, 0.1)


if __name__ == "__main__":
    unittest.main()
