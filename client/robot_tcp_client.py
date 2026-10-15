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
        runtime_log=None,
    ) -> None:
        self._host = host
        self._port = port
        self._timeout_s = timeout_s
        self._reconnect_interval_s = reconnect_interval_s
        self._sock: socket.socket | None = None
        self._rx_buffer = ""
        self._lock = Lock()
        self._runtime_log = runtime_log
        self.last_state_info: dict = {}
        self.last_batch_ack: dict = {}

    def connect(self, retry: bool = True) -> None:
        while True:
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(self._timeout_s)
                sock.connect((self._host, self._port))
                self._sock = sock
                self._rx_buffer = ""
                logger.info("Connected to robot TCP server at %s:%d", self._host, self._port)
                return
            except Exception as exc:  # noqa: BLE001
                if sock is not None:
                    sock.close()
                if not retry:
                    raise
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
            self.connect(retry=False)

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
                result = json.loads(line)
                if self._runtime_log is not None and command == "GET_POLICY_STATUS":
                    if hasattr(self._runtime_log, "sync"):
                        self._runtime_log.sync(result)
                    self._runtime_log.write("robot_status", status=result)
                return result
            except Exception:  # noqa: BLE001
                if command == "GET_POLICY_STATUS" and hasattr(self._runtime_log, "sync"):
                    self._runtime_log.sync({})
                logger.exception("Robot TCP request failed: %s", command)
                self.close()
                raise

    def get_follower_state(self) -> list[float]:
        obj = self._request_json("GET_FOLLOWER_STATE")
        self.last_state_info = {key: value for key, value in obj.items() if key != "q"}
        q = obj.get("q", [])
        if not isinstance(q, list) or len(q) != 7 or not np.isfinite(np.asarray(q, dtype=float)).all():
            raise ValueError(f"Invalid robot state: {obj!r}")
        if obj.get("state_valid") is False:
            raise RuntimeError("Robot feedback has not initialized")
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
            settling = bool(st.get("motion_settling", False))
            if remaining > 0 or settling:
                seen_busy = True
            if remaining == 0 and not settling:
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
                if self._runtime_log is not None:
                    self._runtime_log.write("stop_ack", ack=ack)
            except Exception:  # noqa: BLE001
                logger.exception("Robot TCP STOP_POLICY failed.")
                self.close()
                raise

    def send_policy_actions_batch(self, actions: np.ndarray, obs_age_s: float | None = None,
                                  expected_batch_seq: int | None = None, smooth: bool = False) -> int:
        """发送 ``SET_JOINTS_BATCH`` 一行：``actions`` 为 ``(T, D)`` numpy。返回入队步数。

        ``obs_age_s`` 是从采到 state/图像到这一批入队经过的秒数。控制器按每步 1/30 s
        （30 步共 1s）丢掉已经过期的前缀。
        """
        started = time.monotonic()
        arr = np.asarray(actions, dtype=float)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        if arr.size == 0:
            return 0
        if arr.ndim != 2 or arr.shape[1] != 7 or not np.isfinite(arr).all():
            raise ValueError("Policy actions must be (T, 7) finite numbers")
        if obs_age_s is not None and (not np.isfinite(obs_age_s) or obs_age_s < 0.0):
            raise ValueError("obs_age_s must be finite and nonnegative")
        rows = arr.tolist()
        payload: dict[str, object] = {"actions": rows, "smooth": bool(smooth)}
        if expected_batch_seq is not None:
            payload["expected_batch_seq"] = int(expected_batch_seq)
        with self._lock:
            try:
                self._ensure_connected()
                if obs_age_s is not None:
                    payload["obs_age_s"] = float(obs_age_s) + time.monotonic() - started
                line = "SET_JOINTS_BATCH " + json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n"
                sent_at = time.monotonic()
                self._send_line(line)
                ack_raw = self._recv_line()
                ack = json.loads(ack_raw)
                if not ack.get("accepted", False):
                    raise RuntimeError(f"SET_JOINTS_BATCH rejected by robot: {ack!r}")
                self.last_batch_ack = ack
                if self._runtime_log is not None:
                    self._runtime_log.write("batch_sent", ack=ack, obs_age_s=payload.get("obs_age_s"),
                                            smooth=bool(smooth),
                                            ack_rtt_ms=(time.monotonic() - sent_at) * 1000.0, actions=arr)
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
