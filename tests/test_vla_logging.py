"""Hardware-free checks for persisted VLA bridge timing events."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "client"))

from bridge import BridgeConfig, PiRobotBridge  # noqa: E402
from runtime_logging import configure_logging  # noqa: E402


class FakeCamera:
    def read_rgb(self):
        return np.zeros((2, 3, 3), dtype=np.uint8)


class FakeRobot:
    def get_follower_state(self):
        return [0.0] * 7

    def send_policy_actions_batch(self, actions):
        assert actions.shape == (2, 7)

    def wait_policy_idle(self, **kwargs):
        return 3


class FakePolicy:
    def infer(self, observation):
        assert observation["observation/images/right"].shape == (2, 3, 3)
        return {"actions": np.zeros((2, 7)), "policy_timing": {"infer_ms": 12.5}}


class FailingPolicy:
    def infer(self, observation):
        raise TimeoutError("test timeout")


class VLABridgeLoggingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        configure_logging(self.tmp.name)
        self.cfg = BridgeConfig(
            prompt="test", hz=0, action_dim=7, idle_poll_s=0.005,
            idle_timeout_s=1, start_delay_s=0, post_idle_settle_s=0,
        )

    def tearDown(self):
        self.tmp.cleanup()

    def events(self):
        path = Path(self.tmp.name) / "vla_timing.jsonl"
        return [json.loads(line) for line in path.read_text().splitlines()]

    def test_successful_cycle_records_stage_timings(self):
        bridge = PiRobotBridge(FakePolicy(), FakeRobot(), FakeCamera(), None, self.cfg)
        self.assertTrue(bridge._run_cycle(0))
        result = self.events()[-1]
        self.assertEqual(result["event"], "cycle_end")
        self.assertEqual(result["outcome"], "ok")
        self.assertEqual(result["chunk_len"], 2)
        self.assertEqual(result["status_polls"], 3)
        self.assertEqual(result["image_bytes"], 18)
        self.assertEqual(result["server_infer_ms"], 12.5)
        for stage in ("state", "camera", "inference", "send_batch", "execute_wait"):
            self.assertGreaterEqual(result[f"{stage}_ms"], 0)
        self.assertGreaterEqual(result["cycle_ms"], 0)

    def test_failed_cycle_records_failing_stage(self):
        bridge = PiRobotBridge(FailingPolicy(), FakeRobot(), FakeCamera(), None, self.cfg)
        with self.assertRaises(TimeoutError):
            bridge._run_cycle(1)
        result = self.events()[-1]
        self.assertEqual(result["outcome"], "error")
        self.assertEqual(result["failed_stage"], "inference")
        self.assertEqual(result["error_type"], "TimeoutError")
        self.assertIn("inference_ms", result)


if __name__ == "__main__":
    unittest.main()
