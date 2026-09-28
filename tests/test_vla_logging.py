"""Hardware-free checks for persisted VLA bridge timing events."""

from __future__ import annotations

import json
import logging
import sys
import tempfile
import unittest
from pathlib import Path
from datetime import datetime, timezone
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "client"))

from bridge import BridgeConfig, PiRobotBridge  # noqa: E402
import run_bridge  # noqa: E402
from robot_tcp_client import RobotTcpClient  # noqa: E402
from runtime_logging import PolicyLogSession, configure_logging, record_timing_event  # noqa: E402


class FakeCamera:
    def read_rgb(self):
        return np.zeros((2, 3, 3), dtype=np.uint8)


class FakeRobot:
    def __init__(self):
        self.status = {"policy_running": True, "log_enabled": True, "log_session": "1", "remaining": 0}

    def get_policy_status(self):
        return self.status.copy()

    def get_follower_state(self):
        return [0.0] * 7

    def send_policy_actions_batch(self, actions):
        assert actions.shape == (2, 7)

    def wait_policy_idle(self, **kwargs):
        if kwargs.get("on_status"):
            kwargs["on_status"](self.get_policy_status())
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
        self.session_dir = configure_logging(self.tmp.name)
        self.cfg = BridgeConfig(
            prompt="test", hz=0, action_dim=7, idle_poll_s=0.005,
            idle_timeout_s=1, start_delay_s=0, post_idle_settle_s=0,
        )

    def tearDown(self):
        logging.shutdown()
        self.tmp.cleanup()

    def events(self):
        path = self.session_dir / "vla_timing.jsonl"
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


    def test_restarts_create_distinct_sessions_even_at_same_timestamp(self):
        with patch("runtime_logging.datetime") as clock:
            clock.now.return_value = datetime(2026, 9, 28, 1, 2, 3, tzinfo=timezone.utc)
            first = configure_logging(self.tmp.name)
            record_timing_event("first_run")
            logging.getLogger("bridge").info("first session marker")
            old_text = (first / "vla_bridge.log").read_text()
            old_timing = (first / "vla_timing.jsonl").read_text()
            second = configure_logging(self.tmp.name)
            record_timing_event("second_run")
            logging.getLogger("bridge").info("second session marker")

        self.assertNotEqual(first, second)
        self.assertTrue(first.name.startswith("run_20260928T010203_000000Z_pid"))
        self.assertEqual((first / "vla_bridge.log").read_text(), old_text)
        self.assertEqual((first / "vla_timing.jsonl").read_text(), old_timing)
        self.assertIn("second session marker", (second / "vla_bridge.log").read_text())
        events = [json.loads(line) for line in (second / "vla_timing.jsonl").read_text().splitlines()]
        self.assertEqual([event["event"] for event in events], ["second_run"])

    def test_disabled_logging_creates_no_directory_and_stops_previous_files(self):
        old_text = (self.session_dir / "vla_bridge.log").read_text()
        old_timing = (self.session_dir / "vla_timing.jsonl").read_text()
        unused_dir = Path(self.tmp.name) / "disabled"
        self.assertIsNone(configure_logging(unused_dir, enabled=False))
        bridge = PiRobotBridge(FakePolicy(), FakeRobot(), FakeCamera(), None, self.cfg)
        self.assertTrue(bridge._run_cycle(0))
        self.assertFalse(unused_dir.exists())
        self.assertEqual((self.session_dir / "vla_bridge.log").read_text(), old_text)
        self.assertEqual((self.session_dir / "vla_timing.jsonl").read_text(), old_timing)
        self.assertTrue(any(isinstance(handler, logging.StreamHandler)
                            for handler in logging.getLogger().handlers))
        self.assertFalse(any(isinstance(handler, logging.FileHandler)
                             for handler in logging.getLogger().handlers))

    def test_cli_starts_without_files_and_has_no_logging_switch(self):
        argv = ["run_bridge.py", "--log-dir", self.tmp.name]
        with patch.object(sys, "argv", argv), \
             patch.object(run_bridge, "configure_logging") as configure, \
             patch.object(run_bridge, "run_from_args") as run:
            run_bridge.main()
        configure.assert_called_once_with(self.tmp.name, enabled=False)
        run.assert_called_once()
        self.assertFalse(hasattr(run.call_args.args[0], "log"))

    def test_policy_controls_files_and_restart_without_restarting_bridge(self):
        configure_logging(self.tmp.name, enabled=False)
        root = Path(self.tmp.name) / "policy_logs"
        logs = PolicyLogSession(root)
        robot = FakeRobot()
        robot.status["policy_running"] = False
        bridge = PiRobotBridge(FakePolicy(), robot, FakeCamera(), None, self.cfg, policy_logs=logs)
        bridge._run_cycle(0)
        self.assertFalse(root.exists())

        robot.status["policy_running"] = True
        bridge._run_cycle(1)
        first = logs.session_dir
        bridge._run_cycle(2)
        self.assertEqual(logs.session_dir, first)
        self.assertEqual(len(list(root.iterdir())), 1)

        robot.status["policy_running"] = False
        logs.observe(robot.get_policy_status())
        old_timing = (first / "vla_timing.jsonl").read_text()
        self.assertIsNone(logs.session_dir)

        robot.status.update(policy_running=True, log_enabled=False, log_session="2")
        bridge._run_cycle(3)
        self.assertEqual(len(list(root.iterdir())), 1)
        self.assertEqual((first / "vla_timing.jsonl").read_text(), old_timing)

        robot.status.update(log_enabled=True, log_session="3")
        bridge._run_cycle(4)
        self.assertNotEqual(logs.session_dir, first)
        self.assertEqual(len(list(root.iterdir())), 2)
        self.assertEqual((first / "vla_timing.jsonl").read_text(), old_timing)
        logs.close()

    def test_restart_during_wait_does_not_write_old_cycle_to_new_session(self):
        configure_logging(self.tmp.name, enabled=False)
        logs = PolicyLogSession(Path(self.tmp.name) / "policy_logs")
        robot = FakeRobot()
        bridge = PiRobotBridge(FakePolicy(), robot, FakeCamera(), None, self.cfg, policy_logs=logs)

        def replace_session(**kwargs):
            robot.status["log_session"] = "2"
            kwargs["on_status"](robot.get_policy_status())
            return 1

        with patch.object(robot, "wait_policy_idle", side_effect=replace_session):
            bridge._run_cycle(0)
        new_file = logs.session_dir / "vla_timing.jsonl"
        self.assertNotIn("infer_cycle=0", (logs.session_dir / "vla_bridge.log").read_text())
        events = [json.loads(line) for line in new_file.read_text().splitlines()]
        self.assertEqual([e["event"] for e in events], ["policy_session_start"])
        bridge._run_cycle(1)
        events = [json.loads(line) for line in new_file.read_text().splitlines()]
        cycles = [e["cycle"] for e in events if e["event"] == "cycle_end"]
        self.assertEqual(cycles, [1])
        logs.close()

    def test_robot_polling_delivers_stop_status_to_logger(self):
        configure_logging(self.tmp.name, enabled=False)
        logs = PolicyLogSession(Path(self.tmp.name) / "policy_logs")
        robot = RobotTcpClient("unused", 0)
        running = {"policy_running": True, "log_enabled": True, "log_session": "1", "remaining": 1}
        stopped = dict(running, policy_running=False, log_enabled=False, remaining=0)
        with patch.object(robot, "get_policy_status", side_effect=[running, stopped]):
            self.assertEqual(robot.wait_policy_idle(poll_s=0, on_status=logs.observe), 2)
        self.assertIsNone(logs.session_id)
        files = list((Path(self.tmp.name) / "policy_logs").glob("*/vla_timing.jsonl"))
        events = [json.loads(line) for line in files[0].read_text().splitlines()]
        self.assertEqual([e["event"] for e in events], ["policy_session_start", "policy_session_end"])

    def test_old_robot_status_does_not_enable_file_logging(self):
        configure_logging(self.tmp.name, enabled=False)
        root = Path(self.tmp.name) / "policy_logs"
        logs = PolicyLogSession(root)
        with self.assertLogs("runtime_logging", level="WARNING") as captured:
            logs.sync({"remaining": 0, "idle": True})
            logs.sync({"remaining": 0, "idle": True})
        self.assertEqual(len(captured.records), 1)
        self.assertFalse(root.exists())


if __name__ == "__main__":
    unittest.main()
