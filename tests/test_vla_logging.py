"""Hardware-free regression checks for logging merged into the RTC bridge."""
import json
import logging
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'client'))
from runtime_logging import PolicyLogSession
from robot_tcp_client import RobotTcpClient
from bridge import PiRobotBridge, BridgeConfig


class PolicyLoggingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / 'logs'
        self.logs = PolicyLogSession(str(self.root), {'policy_api_key': 'secret', 'mode': 'async'})
        self.status = dict(policy_running=True, log_enabled=True, log_session='1')

    def tearDown(self):
        self.logs.close()
        self.tmp.cleanup()

    def events(self, path):
        return [json.loads(line) for line in (path / 'events.jsonl').read_text().splitlines()]

    def test_no_files_before_policy_or_when_logging_disabled(self):
        self.logs.write('idle')
        for status in ({}, dict(self.status, policy_running=False), dict(self.status, log_enabled=False)):
            self.logs.sync(status)
        self.assertFalse(self.root.exists())

    def test_policy_restart_rotates_files_and_preserves_rtc_schema(self):
        self.logs.sync(self.status)
        first = self.logs.session_dir
        self.logs.write('inference', actions=np.zeros((2, 7)))
        self.logs.sync(self.status)
        self.assertEqual(self.logs.session_dir, first)
        self.logs.sync(dict(self.status, policy_running=False))
        old = (first / 'events.jsonl').read_text()
        self.logs.sync(dict(self.status, log_session='2'))
        second = self.logs.session_dir
        self.assertNotEqual(first, second)
        self.assertEqual((first / 'events.jsonl').read_text(), old)
        records = self.events(first)
        self.assertEqual(records[0]['config']['mode'], 'async')
        self.assertNotIn('policy_api_key', records[0]['config'])
        self.assertIn('actions', next(r for r in records if r['event'] == 'inference'))
        self.assertEqual(records[-1]['event'], 'session_end')
        self.assertEqual(self.events(second)[1]['event'], 'policy_session_start')

    def test_disabled_and_missing_status_close_files(self):
        for status in (dict(self.status, log_enabled=False), {}, dict(self.status, policy_running=False)):
            self.logs.sync(self.status)
            path = self.logs.session_dir
            self.logs.sync(status)
            old = (path / 'events.jsonl').read_text()
            self.logs.write('must_not_write')
            self.assertIsNone(self.logs.session_id)
            self.assertEqual((path / 'events.jsonl').read_text(), old)

    def test_status_request_applies_log_state_before_recording(self):
        robot = RobotTcpClient('unused', 0, runtime_log=self.logs)
        statuses = [self.status, dict(self.status, policy_running=False)]
        with patch.object(robot, '_send_line'), patch.object(robot, '_recv_line', side_effect=[json.dumps(s) for s in statuses]):
            robot.get_policy_status()
            path = self.logs.session_dir
            robot.get_policy_status()
        self.assertIsNone(self.logs.session_dir)
        self.assertEqual(sum(e['event'] == 'robot_status' for e in self.events(path)), 1)

    def test_status_failure_closes_files(self):
        self.logs.sync(self.status)
        robot = RobotTcpClient('unused', 0, runtime_log=self.logs)
        with patch.object(robot, '_send_line', side_effect=ConnectionError('offline')):
            with self.assertRaises(ConnectionError):
                robot.get_policy_status()
        self.assertIsNone(self.logs.session_id)

    def test_concurrent_status_and_writes_close_without_reopening(self):
        self.logs.sync(self.status)
        errors = []
        def writer():
            try:
                for _ in range(100):
                    self.logs.write('sample', values=np.zeros(7))
            except Exception as exc:
                errors.append(exc)
        thread = threading.Thread(target=writer)
        thread.start()
        self.logs.sync(dict(self.status, log_session='2'))
        self.logs.close()
        thread.join()
        self.logs.sync(self.status)
        self.assertEqual(errors, [])
        self.assertIsNone(self.logs.session_id)
        self.assertEqual(sum(isinstance(h, logging.FileHandler) for h in logging.getLogger().handlers), 0)

    def test_inference_crossing_restart_does_not_log_into_new_session(self):
        from types import SimpleNamespace
        self.logs.sync(self.status)
        def infer(obs):
            self.logs.sync(dict(self.status, log_session='2'))
            return {'actions': np.zeros((2, 7))}
        robot = SimpleNamespace(last_state_info={})
        camera = SimpleNamespace(read_rgb=lambda: np.zeros((224, 224, 3), dtype=np.uint8), last_frame_info={})
        cfg = BridgeConfig(prompt='test', hz=0, action_dim=7, idle_poll_s=0, idle_timeout_s=1,
                           start_delay_s=0, post_idle_settle_s=0)
        bridge = PiRobotBridge(SimpleNamespace(infer=infer), robot, camera, None, cfg, runtime_log=self.logs)
        self.assertEqual(bridge._infer_chunk(np.zeros(7)).shape, (2, 7))
        self.assertNotIn('inference', [e['event'] for e in self.events(self.logs.session_dir)])


if __name__ == '__main__':
    unittest.main()
