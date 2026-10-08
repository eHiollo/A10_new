import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "client"))

from bridge import BridgeConfig, PiRobotBridge, _extract_action_chunk, _state_7d, run_from_args, build_arg_parser
from robot_tcp_client import RobotTcpClient
from runtime_logging import RuntimeLog
import run_bridge
from usb_camera import CameraConfig, ThreadedCamera
from analyze_policy_log import analyze_retargets


class PolicyClientTests(unittest.TestCase):
    def test_policy_xml_declares_smoothing_acceleration(self):
        root = ET.parse(Path(__file__).resolve().parents[1] / "kaanh.xml").getroot()
        command = root.find(".//A10PolicyTcpDriver/Command")
        self.assertIsNotNone(command)
        params = {param.attrib["name"]: param.attrib["default"] for param in command.iter("Param")}
        self.assertEqual(float(params["arm_acc"]), 0.5)
        self.assertEqual(float(params["arm_vel"]), 0.05)

    def test_async_smoothing_switch_and_wire(self):
        self.assertTrue(build_arg_parser().parse_args([]).async_smoothing)
        self.assertFalse(build_arg_parser().parse_args(["--no-async-smoothing"]).async_smoothing)
        client = RobotTcpClient("unused", 0)
        with mock.patch.object(client, "_ensure_connected"), mock.patch.object(client, "_send_line") as sender, \
                mock.patch.object(client, "_recv_line", return_value='{"accepted":true,"queued":2}'):
            for smooth in (True, False):
                client.send_policy_actions_batch(np.zeros((2, 7)), smooth=smooth)
                payload = json.loads(sender.call_args.args[0].split(" ", 1)[1])
                self.assertIs(payload["smooth"], smooth)

    def test_async_smoothing_requires_controller_capability(self):
        for supported, enabled in ((True, True), (False, True), (True, False)):
            camera = SimpleNamespace(read_rgb=lambda: np.zeros((224, 224, 3), dtype=np.uint8))
            robot = mock.Mock()
            robot.get_follower_state.return_value = [0.0] * 7
            robot.get_policy_status.return_value = {"remaining": 3, "policy_running": True,
                                                    "supports_async_smoothing": supported}
            websocket = mock.Mock()
            websocket.infer.return_value = {"actions": np.zeros((30, 7))}
            config = BridgeConfig("test", 10, 7, 0.005, 1.0, 0.0, 0.0, async_smoothing=enabled)
            bridge = PiRobotBridge(websocket, robot, camera, None, config)
            robot.send_policy_actions_batch.side_effect = lambda *args, **kwargs: bridge.stop() or 30
            bridge.run_forever()
            bridge._infer_thread.join(timeout=1.0)
            self.assertIs(robot.send_policy_actions_batch.call_args.kwargs["smooth"], supported and enabled)

    def test_idle_wait_includes_smoother_settling(self):
        client = RobotTcpClient("unused", 0)
        statuses = [{"remaining": 0, "motion_settling": True},
                    {"remaining": 0, "motion_settling": False}]
        with mock.patch.object(client, "get_policy_status", side_effect=statuses) as status:
            self.assertEqual(client.wait_policy_idle(poll_s=0.02, timeout_s=1.0), "ok")
            self.assertEqual(status.call_count, 2)

    def test_company_camera_mapping_is_default(self):
        args = build_arg_parser().parse_args([])
        self.assertEqual(args.top_camera, "2")
        self.assertEqual(args.right_camera, "0")
        self.assertTrue(args.right_rotate_180)
        self.assertFalse(args.top_rotate_180)

    def test_log_analysis_distinguishes_model_direction_from_delayed_pullback(self):
        actions = [[-0.002] * 7, [-0.003] * 7]
        prediction = {"state": [0.0] * 7, "actions": actions}
        sent = {"actions": actions, "ack": {"batch_seq": 9}}
        event = {"event": "retarget", "batch_seq": 9, "prefix_skip": 1,
                 "q0": [-0.015] * 7, "q1": actions[1], "n_steps": 121}
        report = analyze_retargets([prediction], [sent], [event])
        self.assertEqual(report["matched_batches"], 1)
        self.assertEqual(report["opposite_to_model_delta_per_axis"], [1] * 6)
        self.assertAlmostEqual(report["recent_batches"][0]["applied_delta"][3], 0.012)
        self.assertEqual(analyze_retargets([], [], [event])["matched_batches"], 0)

    def test_user_startup_command_creates_session(self):
        with tempfile.TemporaryDirectory() as directory:
            argv = ["run_bridge.py", "--policy-host", "192.168.110.119", "--policy-port", "8000",
                    "--robot-host", "127.0.0.1", "--robot-port", "8080", "--hz", "30", "--log-dir", directory]
            with mock.patch.object(sys, "argv", argv), mock.patch.object(run_bridge, "run_from_args") as runner:
                run_bridge.main()
            args = runner.call_args.args[0]
            self.assertEqual(args.hz, 30)
            self.assertEqual(args.log_dir, directory)
            records = [json.loads(line) for line in next(Path(directory).glob("run_*/events.jsonl")).read_text().splitlines()]
            self.assertEqual([record["event"] for record in records], ["session_start", "session_end"])

    def test_invalid_actions_and_observations_are_rejected(self):
        for actions in (np.zeros((0, 7)), np.full((2, 7), np.nan), np.full((2, 7), np.inf)):
            with self.assertRaises(ValueError):
                _extract_action_chunk({"actions": actions})
        for state in ([], [0.0] * 6, [float("nan")] * 7):
            with self.assertRaises(ValueError):
                _state_7d(state)

    def test_camera_age_is_diagnostic_only(self):
        camera = ThreadedCamera.__new__(ThreadedCamera)
        camera._cfg = CameraConfig(source="2", max_frame_age_s=0.5)
        camera._frame_lock = threading.Lock()
        camera._latest = np.zeros((2, 2, 3), dtype=np.uint8)
        camera._latest_at = time.monotonic() - 1.0
        camera._open_error = None
        self.assertIs(camera.read_rgb(), camera._latest)
        self.assertTrue(camera.last_frame_info["stale"])
        camera._latest_at = time.monotonic()
        self.assertIs(camera.read_rgb(), camera._latest)
        self.assertLess(camera.last_frame_info["age_s"], 0.5)

    def test_batch_ack_and_wire_age(self):
        client = RobotTcpClient("unused", 0)
        with mock.patch.object(client, "_ensure_connected"), mock.patch.object(client, "_send_line") as sender, \
                mock.patch.object(client, "_recv_line", return_value='{"accepted":true,"queued":2,"batch_seq":42}'):
            self.assertEqual(client.send_policy_actions_batch(np.zeros((2, 7)), obs_age_s=6.0, expected_batch_seq=41), 2)
            payload = json.loads(sender.call_args.args[0].split(" ", 1)[1])
            self.assertGreaterEqual(payload["obs_age_s"], 6.0)
            self.assertEqual(payload["expected_batch_seq"], 41)
            self.assertEqual(client.last_batch_ack["batch_seq"], 42)
            with self.assertRaises(ValueError):
                client.send_policy_actions_batch(np.full((1, 7), np.nan))
            self.assertEqual(sender.call_count, 1)

    def test_logs_include_model_inputs_without_api_key(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime_log = RuntimeLog(directory, {"policy_api_key": "test-secret", "mode": "async"})
            runtime_log.save_observation_images({"observation/images/right": np.zeros((224, 224, 3), dtype=np.uint8)})
            runtime_log.write("inference", state=np.zeros(7), actions=np.zeros((30, 7)))
            runtime_log.close()
            contents = (runtime_log.path / "events.jsonl").read_text()
            self.assertNotIn("test-secret", contents)
            self.assertTrue((runtime_log.path / "policy_right.png").exists())
            record = json.loads(contents.splitlines()[1])
            self.assertEqual(len(record["actions"]), 30)

    def test_inference_failure_retries_without_exiting_bridge(self):
        camera = SimpleNamespace(read_rgb=lambda: np.zeros((224, 224, 3), dtype=np.uint8))
        robot = mock.Mock()
        robot.get_follower_state.return_value = [0.0] * 7
        robot.get_policy_status.return_value = {"remaining": 0, "policy_protocol_version": 2, "policy_running": True}
        websocket = mock.Mock()
        config = BridgeConfig("test", 30, 7, 0.005, 1.0, 0.0, 0.0)
        bridge = PiRobotBridge(websocket, robot, camera, None, config)
        attempts = 0

        def infer(observation):
            nonlocal attempts
            attempts += 1
            if attempts == 1:
                raise RuntimeError("model unavailable")
            return {"actions": np.zeros((10, 7))}

        websocket.infer.side_effect = infer
        robot.send_policy_actions_batch.side_effect = lambda *args, **kwargs: bridge.stop() or 10
        bridge.run_forever()
        bridge._infer_thread.join(timeout=1.0)
        self.assertTrue(bridge._stop.is_set())
        self.assertGreaterEqual(attempts, 2)
        robot.send_policy_actions_batch.assert_called_once()

    def test_bridge_stays_alive_across_policy_stop_and_restart(self):
        camera = SimpleNamespace(read_rgb=lambda: np.zeros((224, 224, 3), dtype=np.uint8))
        robot = mock.Mock()
        robot.get_follower_state.return_value = [0.0] * 7
        running = threading.Event()
        robot.get_policy_status.side_effect = lambda: {"remaining": 0, "policy_running": running.is_set()}
        websocket = mock.Mock()

        def infer(observation):
            time.sleep(0.01)
            return {"actions": np.zeros((10, 7))}

        websocket.infer.side_effect = infer
        robot.send_policy_actions_batch.return_value = 10
        config = BridgeConfig("test", 30, 7, 0.005, 1.0, 0.0, 0.0)
        bridge = PiRobotBridge(websocket, robot, camera, None, config)
        worker = threading.Thread(target=bridge.run_forever, daemon=True)
        worker.start()
        try:
            time.sleep(0.08)
            self.assertTrue(worker.is_alive())
            self.assertGreater(websocket.infer.call_count, 0)
            robot.send_policy_actions_batch.assert_not_called()
            for iteration in range(2):
                sent_before = robot.send_policy_actions_batch.call_count
                running.set()
                deadline = time.monotonic() + 1.0
                while robot.send_policy_actions_batch.call_count == sent_before and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertGreater(robot.send_policy_actions_batch.call_count, sent_before)
                running.clear()
                time.sleep(0.05)
                sent_after = robot.send_policy_actions_batch.call_count
                time.sleep(0.05)
                self.assertEqual(robot.send_policy_actions_batch.call_count, sent_after)
                self.assertTrue(worker.is_alive())
        finally:
            bridge.stop()
            worker.join(timeout=2.0)

    def test_shutdown_sends_stop_after_non_keyboard_error(self):
        args = build_arg_parser().parse_args(["--top-camera", ""])
        with mock.patch("bridge.OpenPIWebsocketClient") as websocket, mock.patch("bridge.RobotTcpClient") as robot, \
                mock.patch("bridge._make_camera") as camera, mock.patch("bridge.PiRobotBridge") as bridge:
            bridge.return_value._infer_thread = None
            bridge.return_value.run_forever.side_effect = RuntimeError("camera failed")
            with self.assertRaisesRegex(RuntimeError, "camera failed"):
                run_from_args(args)
            robot.return_value.stop_policy.assert_called_once()
            robot.return_value.close.assert_called_once()
            websocket.return_value.close.assert_called_once()
            camera.return_value.close.assert_called_once()

    def test_pipeline_infers_before_empty_and_sends_only_at_boundary(self):
        camera = SimpleNamespace(read_rgb=lambda: np.zeros((224, 224, 3), dtype=np.uint8))
        robot = mock.Mock()
        robot.get_follower_state.return_value = [0.0] * 7
        active = {"policy_running": True, "batch_seq": 7, "remaining": 12, "remaining_time_s": 0.30}
        idle = {**active, "remaining": 0, "remaining_time_s": 0.0}
        statuses = [active, {**active, "remaining": 5}, idle]
        robot.get_policy_status.side_effect = lambda: statuses.pop(0) if statuses else idle
        websocket = mock.Mock()

        def infer(observation):
            self.assertEqual(len(statuses), 2)
            self.assertGreater(statuses[0]["remaining"], 0)
            robot.send_policy_actions_batch.assert_not_called()
            return {"actions": np.zeros((30, 7))}

        websocket.infer.side_effect = infer
        config = BridgeConfig("test", 30, 7, 0.005, 1.0, 0.0, 0.0, mode="pipeline")
        bridge = PiRobotBridge(websocket, robot, camera, None, config)

        def send(*args, **kwargs):
            self.assertEqual(statuses, [])
            self.assertEqual(kwargs["expected_batch_seq"], 7)
            bridge.stop()
            return 30

        robot.send_policy_actions_batch.side_effect = send
        bridge._run_pipeline()
        websocket.infer.assert_called_once()
        robot.send_policy_actions_batch.assert_called_once()

    def test_pipeline_discards_prediction_after_stop_restart_or_expiry(self):
        robot = mock.Mock()
        config = BridgeConfig("test", 30, 7, 0.005, 1.0, 0.0, 0.0, mode="pipeline")
        bridge = PiRobotBridge(mock.Mock(), robot, mock.Mock(), None, config)
        for status, deadline in (
            ({"policy_running": False, "batch_seq": 7, "remaining": 0}, time.perf_counter() + 1),
            ({"policy_running": True, "batch_seq": 9, "remaining": 0}, time.perf_counter() + 1),
            ({"policy_running": True, "batch_seq": 7, "remaining": 0}, time.perf_counter() - 1),
        ):
            robot.get_policy_status.return_value = status
            self.assertIsNone(bridge._wait_pipeline_boundary(7, deadline))
        robot.send_policy_actions_batch.assert_not_called()

    def test_pipeline_waits_for_execution_time_not_keyframe_count(self):
        robot = mock.Mock()
        far = {"policy_running": True, "batch_seq": 7, "remaining": 1, "remaining_time_s": 2.0}
        near = {**far, "remaining_time_s": 0.4}
        robot.get_policy_status.side_effect = [far, near]
        config = BridgeConfig("test", 30, 7, 0.005, 1.0, 0.0, 0.0, mode="pipeline")
        bridge = PiRobotBridge(mock.Mock(), robot, mock.Mock(), None, config)
        self.assertEqual(bridge._wait_pipeline_window(0.45), near)

    def test_sync_executes_full_chunk_without_age_cropping(self):
        camera = SimpleNamespace(read_rgb=lambda: np.zeros((224, 224, 3), dtype=np.uint8), last_frame_info={})
        robot = mock.Mock()
        robot.get_follower_state.return_value = [0.0] * 7
        robot.get_policy_status.return_value = {"remaining": 0, "policy_running": True}
        websocket = mock.Mock()
        websocket.infer.return_value = {"actions": np.zeros((30, 7))}
        config = BridgeConfig("test", 0, 7, 0.005, 1.0, 0.0, 0.0, mode="sync")
        bridge = PiRobotBridge(websocket, robot, camera, None, config)

        def send(actions, obs_age_s=None, **kwargs):
            self.assertEqual(actions.shape, (30, 7))
            self.assertIsNone(obs_age_s)
            self.assertFalse(kwargs.get("smooth", False))
            bridge.stop()
            return 30

        robot.send_policy_actions_batch.side_effect = send
        robot.wait_policy_idle.return_value = "stopped"
        bridge._run_sync()
        robot.send_policy_actions_batch.assert_called_once()


if __name__ == "__main__":
    unittest.main()
