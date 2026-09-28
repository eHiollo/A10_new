from __future__ import annotations

import json
import logging
import socket
import time
from threading import Lock

import numpy as np

logger = logging.getLogger(__name__)


class RobotTcpClient:
    """Client for ../a10_tcp_server.cpp line-based protocol."""

    def __init__(
        self,
        host: str,
        port: int,
        timeout_s: float = 1.0,
        reconnect_interval_s: float = 1.0,
    ) -> None:
        self._host = host
        self._port = port
        self._timeout_s = timeout_s
        self._reconnect_interval_s = reconnect_interval_s
        self._sock: socket.socket | None = None
        self._rx_buffer = ""
        self._lock = Lock()

    def connect(self) -> None:
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(self._timeout_s)
                sock.connect((self._host, self._port))
                self._sock = sock
                self._rx_buffer = ""
                logger.info("Connected to robot TCP server at %s:%d", self._host, self._port)
                return
            except Exception as exc:  # noqa: BLE001
                logger.warning("Robot TCP connect failed (%s), retrying...", exc)
                time.sleep(self._reconnect_interval_s)

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:  # noqa: BLE001
                pass
            self._sock = None

    def _ensure_connected(self) -> None:
        if self._sock is None:
            self.connect()

    def _send_line(self, line: str) -> None:
        self._ensure_connected()
        assert self._sock is not None
        payload = line.encode("utf-8")
        self._sock.sendall(payload)

    def _recv_line(self) -> str:
        self._ensure_connected()
        assert self._sock is not None

        while True:
            pos = self._rx_buffer.find("\n")
            if pos >= 0:
                line = self._rx_buffer[:pos]
                self._rx_buffer = self._rx_buffer[pos + 1 :]
                return line

            data = self._sock.recv(4096)
            if not data:
                raise ConnectionError("Robot TCP socket closed by peer.")
            self._rx_buffer += data.decode("utf-8", errors="ignore")

    def _request_json(self, command: str) -> dict:
        with self._lock:
            try:
                self._send_line(f"{command}\n")
                line = self._recv_line()
                return json.loads(line)
            except Exception:  # noqa: BLE001
                logger.exception("Robot TCP request failed: %s", command)
                self.close()
                raise

    def get_follower_state(self) -> list[float]:
        obj = self._request_json("GET_FOLLOWER_STATE")
        q = obj.get("q", [])
        if not isinstance(q, list):
            return []
        return [float(v) for v in q]

    def get_leader_state(self) -> list[float]:
        obj = self._request_json("GET_LEADER_STATE")
        q = obj.get("q", [])
        if not isinstance(q, list):
            return []
        return [float(v) for v in q]

    def send_joint_targets(self, q: list[float]) -> None:
        payload = {"q": [float(v) for v in q]}
        line = json.dumps(payload, ensure_ascii=True) + "\n"
        with self._lock:
            try:
                self._send_line(line)
            except Exception:  # noqa: BLE001
                logger.exception("Robot TCP send joint target failed.")
                self.close()
                raise

    def get_policy_status(self) -> dict:
        """查询 ``target_q_batch_`` 是否已执行完：``{"idle": bool, "remaining": int}``。"""
        return self._request_json("GET_POLICY_STATUS")

    def wait_policy_idle(
        self,
        poll_s: float = 0.005,
        timeout_s: float = 120.0,
        should_stop=None,
        expected_queued: int | None = None,
    ) -> str:
        """轮询直到 ``remaining==0``。返回 ``ok`` / ``skipped`` / ``stopped``。

        驱动若在一个实时周期内把整段丢掉，remaining 会在几毫秒内变成 0。
        若从未看到 remaining>0，或耗时短于 ``queued * 2 / 500Hz`` 的一半，判为 skip。
        """
        t0 = time.time()
        seen_busy = False
        min_exec_s = 0.0
        if expected_queued is not None and int(expected_queued) > 0:
            min_exec_s = int(expected_queued) * 2.0 / 500.0
        while time.time() - t0 < timeout_s:
            if should_stop is not None and should_stop():
                return "stopped"
            st = self.get_policy_status()
            rem = st.get("remaining")
            if rem is None:
                raise RuntimeError(f"GET_POLICY_STATUS missing 'remaining': {st!r}")
            try:
                remaining = int(rem)
            except (TypeError, ValueError) as exc:
                raise RuntimeError(f"GET_POLICY_STATUS 'remaining' not int: {st!r}") from exc
            if remaining > 0:
                seen_busy = True
            if remaining == 0:
                elapsed = time.time() - t0
                too_fast = elapsed < max(0.015, min_exec_s * 0.5)
                if too_fast or not seen_busy:
                    logger.error(
                        "policy batch skipped: remaining=0 elapsed=%.1fms queued=%s seen_busy=%s status=%s",
                        elapsed * 1000.0,
                        expected_queued,
                        seen_busy,
                        st,
                    )
                    return "skipped"
                return "ok"
            time.sleep(poll_s)
        raise TimeoutError(f"wait_policy_idle exceeded {timeout_s}s")

    def stop_policy(self) -> None:
        """发送 ``STOP_POLICY``：清空 batch 队列并请求 policy 驱动退出，停在当前位置。"""
        with self._lock:
            try:
                self._send_line("STOP_POLICY\n")
                ack_raw = self._recv_line()
                ack = json.loads(ack_raw)
                if not ack.get("stopped", False):
                    raise RuntimeError(f"STOP_POLICY not acknowledged by robot: {ack!r}")
            except Exception:  # noqa: BLE001
                logger.exception("Robot TCP STOP_POLICY failed.")
                self.close()
                raise

    def send_policy_actions_batch(self, actions: np.ndarray) -> int:
        """发送 ``SET_JOINTS_BATCH`` 一行：``actions`` 为 ``(T, D)`` numpy。返回入队步数。"""
        arr = np.asarray(actions, dtype=float)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        if arr.size == 0:
            return 0
        rows = arr.tolist()
        payload = {"actions": rows}
        line = "SET_JOINTS_BATCH " + json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        with self._lock:
            try:
                self._send_line(line)
                ack_raw = self._recv_line()
                ack = json.loads(ack_raw)
                if not ack.get("accepted", False):
                    raise RuntimeError(f"SET_JOINTS_BATCH rejected by robot: {ack!r}")
                return int(ack.get("queued", 0))
            except Exception:  # noqa: BLE001
                logger.exception("Robot TCP SET_JOINTS_BATCH failed.")
                self.close()
                raise

    def send_joint_targets_batch(self, rows: list[list[float]]) -> None:
        """兼容旧名：等价于 ``send_policy_actions_batch``。"""
        if not rows:
            return
        self.send_policy_actions_batch(np.asarray(rows, dtype=float))
