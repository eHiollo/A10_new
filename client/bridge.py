from __future__ import annotations

import argparse
import logging
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any

import numpy as np

from openpi_ws_client import OpenPIWebsocketClient
from robot_tcp_client import RobotTcpClient
from runtime_logging import record_timing_event
from usb_camera import CameraConfig, USBCamera

logger = logging.getLogger(__name__)


def _state_7d(values: list[float]) -> np.ndarray:
    """``(7,) float32``：关节 0–5 为绝对角（rad），第 6 维为夹爪绝对开度（mm）。

    不做归一化，quantile 统计在推理服务端。
    """
    arr = np.asarray(values, dtype=np.float32).reshape(-1)
    if arr.shape[0] >= 7:
        out = arr[:7]
    else:
        out = np.concatenate([arr, np.zeros((7 - arr.shape[0],), dtype=np.float32)], axis=0)
    return np.ascontiguousarray(out, dtype=np.float32)


def _extract_action_chunk(result: dict[str, Any], expected_dim: int = 7) -> np.ndarray:
    if "actions" not in result:
        raise KeyError(f"OpenPI response missing 'actions'. keys={list(result.keys())}")

    actions = np.asarray(result["actions"], dtype=np.float32)
    if actions.ndim == 1:
        actions = actions[None, :]
    elif actions.ndim == 3:
        if actions.shape[0] != 1:
            raise ValueError(f"Unexpected action shape: {actions.shape}")
        actions = actions[0]
    elif actions.ndim != 2:
        raise ValueError(f"Unexpected action shape: {actions.shape}")

    if actions.shape[-1] < expected_dim:
        raise ValueError(f"Action dim too small: {actions.shape[-1]}, expected >= {expected_dim}")

    if actions.shape[-1] > expected_dim:
        actions = actions[:, :expected_dim]

    # 服务端已把预测增量加回当前 state，并裁成 (T, 7)。这里原样下发，不要再加 state。
    return np.ascontiguousarray(actions, dtype=np.float32)


@dataclass
class BridgeConfig:
    prompt: str
    hz: float
    action_dim: int
    idle_poll_s: float
    idle_timeout_s: float
    start_delay_s: float
    post_idle_settle_s: float


class PiRobotBridge:
    def __init__(
        self,
        ws_client: OpenPIWebsocketClient,
        robot_client: RobotTcpClient,
        right_cam: USBCamera,
        top_cam: USBCamera | None,
        cfg: BridgeConfig,
    ) -> None:
        self._ws = ws_client
        self._robot = robot_client
        self._right_cam = right_cam
        self._top_cam = top_cam
        self._cfg = cfg

    def _build_observation(self, state_7d: np.ndarray) -> dict[str, Any]:
        right_rgb = USBCamera.as_policy_hwc(self._right_cam.read_rgb())
        obs: dict[str, Any] = {
            "observation/state": state_7d,
            "observation/images/right": right_rgb,
            "prompt": self._cfg.prompt,
        }
        # 缺 top 时不要用右腕画面顶上。服务端会把缺的那路填成黑图且 mask=false。
        if self._top_cam is not None:
            obs["observation/images/top"] = USBCamera.as_policy_hwc(self._top_cam.read_rgb())
        return obs

    def run_forever(self) -> None:
        """推理、下发动作、等待机器人执行；持续记录每轮的阶段耗时。"""
        infer_period_s = 1.0 / self._cfg.hz if self._cfg.hz > 0 else 0.0
        infer_cycle = 0
        if self._cfg.start_delay_s > 0:
            logger.info("启动延时 %.3fs：延时结束后开始发送 obs / 推理 / 执行动作。", self._cfg.start_delay_s)
            time.sleep(self._cfg.start_delay_s)
        while True:
            t_cycle = time.perf_counter()
            try:
                if not self._run_cycle(infer_cycle):
                    time.sleep(0.05)
                    continue
            except Exception:  # noqa: BLE001
                logger.exception("Bridge step failed. Retrying next cycle.")
                time.sleep(0.2)
            finally:
                # Every attempt gets a unique ID, including errors and empty responses.
                infer_cycle += 1

            if infer_period_s > 0:
                elapsed = time.perf_counter() - t_cycle
                remain = infer_period_s - elapsed
                if remain > 0:
                    time.sleep(remain)

    def _run_cycle(self, infer_cycle: int) -> bool:
        """记录一轮端到端耗时；异常时也保留已完成阶段及失败阶段。"""
        started = time.perf_counter()
        metrics: dict[str, int | float | str] = {"cycle": infer_cycle}
        stage_name = "start"
        outcome = "error"
        record_timing_event("cycle_start", cycle=infer_cycle)

        @contextmanager
        def stage(name: str):
            nonlocal stage_name
            stage_name = name
            stage_started = time.perf_counter()
            record_timing_event("stage_start", cycle=infer_cycle, stage=name)
            try:
                yield
            finally:
                metrics[f"{name}_ms"] = round((time.perf_counter() - stage_started) * 1000, 3)

        try:
            with stage("state"):
                robot_state = self._robot.get_follower_state()
                state_7d = _state_7d(robot_state)
            with stage("camera"):
                obs = self._build_observation(state_7d)
                metrics["image_bytes"] = sum(
                    image.nbytes for key, image in obs.items() if key.startswith("observation/images/")
                )
            with stage("inference"):
                infer_result = self._ws.infer(obs)
                action_chunk = _extract_action_chunk(infer_result, expected_dim=self._cfg.action_dim)
            policy_timing = infer_result.get("policy_timing") or {}
            if isinstance(policy_timing, dict) and isinstance(policy_timing.get("infer_ms"), (int, float)):
                metrics["server_infer_ms"] = policy_timing["infer_ms"]
            n = int(action_chunk.shape[0])
            metrics["chunk_len"] = n
            if n <= 0:
                outcome = "empty_actions"
                logger.warning("推理返回的 action 序列为空，跳过本轮。")
                return False

            with stage("send_batch"):
                self._robot.send_policy_actions_batch(action_chunk)
            with stage("execute_wait"):
                metrics["status_polls"] = self._robot.wait_policy_idle(
                    poll_s=self._cfg.idle_poll_s,
                    timeout_s=self._cfg.idle_timeout_s,
                )
            if self._cfg.post_idle_settle_s > 0:
                # batch 期间状态刷新可能暂停；等待一次刷新窗口。
                with stage("settle"):
                    time.sleep(self._cfg.post_idle_settle_s)

            if infer_cycle % 20 == 0:
                logger.info(
                    "infer_cycle=%d chunk_len=%d infer_ms=%s state=%s action[0]=%s",
                    infer_cycle,
                    n,
                    metrics.get("server_infer_ms"),
                    np.array2string(state_7d, precision=4, suppress_small=True),
                    np.array2string(action_chunk[0], precision=4, suppress_small=True),
                )
            outcome = "ok"
            return True
        except Exception as exc:
            metrics["failed_stage"] = stage_name
            metrics["error_type"] = type(exc).__name__
            raise
        finally:
            metrics["cycle_ms"] = round((time.perf_counter() - started) * 1000, 3)
            record_timing_event("cycle_end", outcome=outcome, **metrics)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Bridge OpenPI websocket inference with this repo's a10_tcp_server (TCP).")
    parser.add_argument("--policy-host", type=str, default="192.168.110.119")
    parser.add_argument("--policy-port", type=int, default=8000)
    parser.add_argument("--policy-api-key", type=str, default=None)
    parser.add_argument("--robot-host", type=str, default="127.0.0.1")
    parser.add_argument(
        "--robot-port",
        type=int,
        default=8080,
        help="机器人 TCP 端口（与 main.cpp 中 tcp_server.start(8080) 一致）。",
    )
    parser.add_argument(
        "--right-camera",
        type=str,
        default="2",
        help="右腕 / wrist 相机（默认 video2，即索引 2 或 /dev/video2）。",
    )
    parser.add_argument(
        "--top-camera",
        type=str,
        default="0",
        help="顶视 / third-person 相机（默认 video0），对应 observation/images/top。传空字符串则不发这一路，服务端按没看到处理。",
    )
    parser.add_argument(
        "--right-rotate-180",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="右相机画面旋转 180° 再送入策略（右腕倒装，默认开启）。",
    )
    parser.add_argument(
        "--top-rotate-180",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="顶视相机画面旋转 180° 再送入策略（默认关闭）。",
    )
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--prompt", type=str, default="pick and place")
    parser.add_argument(
        "--hz",
        type=float,
        default=10.0,
        help="整轮（推理 + batch 执行完 + wait idle）之间的节流：每轮结束后至少间隔 1/hz 秒；0 为不节流。",
    )
    parser.add_argument("--idle-poll", type=float, default=0.005, help="轮询 GET_POLICY_STATUS 的间隔（秒）。")
    parser.add_argument("--idle-timeout", type=float, default=120.0, help="等待 batch 执行完的最长时间（秒）。")
    parser.add_argument(
        "--post-idle-settle",
        type=float,
        default=0.05,
        help="检测到 remaining==0 后额外等待的时间（秒），用于等待状态线程刷新 robot_q_。",
    )
    parser.add_argument("--action-dim", type=int, default=7)
    parser.add_argument(
        "--log-dir",
        type=str,
        default="logs",
        help="运行日志目录（默认当前工作目录下的 logs；日志按大小轮转）。",
    )
    parser.add_argument(
        "--start-delay",
        type=float,
        default=1.0,
        help="启动后在首次发送 obs / 推理 / 执行动作前等待的秒数（默认 1 秒）。",
    )
    return parser


def run_from_args(args: argparse.Namespace) -> None:
    ws_client = OpenPIWebsocketClient(
        host=args.policy_host,
        port=args.policy_port,
        api_key=args.policy_api_key,
    )
    robot_client = RobotTcpClient(host=args.robot_host, port=args.robot_port)
    robot_client.connect()

    use_dual_cam = isinstance(args.top_camera, str) and args.top_camera.strip()
    exclusive_per_read = bool(use_dual_cam)

    right_cam = USBCamera(
        CameraConfig(
            source=args.right_camera,
            width=args.camera_width,
            height=args.camera_height,
            rotate_180=bool(args.right_rotate_180),
            exclusive_per_read=exclusive_per_read,
        )
    )
    top_cam = None
    if use_dual_cam:
        top_cam = USBCamera(
            CameraConfig(
                source=args.top_camera.strip(),
                width=args.camera_width,
                height=args.camera_height,
                rotate_180=bool(args.top_rotate_180),
                exclusive_per_read=exclusive_per_read,
            )
        )

    bridge = PiRobotBridge(
        ws_client=ws_client,
        robot_client=robot_client,
        right_cam=right_cam,
        top_cam=top_cam,
        cfg=BridgeConfig(
            prompt=args.prompt,
            hz=float(args.hz),
            action_dim=int(args.action_dim),
            idle_poll_s=float(args.idle_poll),
            idle_timeout_s=float(args.idle_timeout),
            start_delay_s=max(0.0, float(args.start_delay)),
            post_idle_settle_s=max(0.0, float(args.post_idle_settle)),
        ),
    )

    try:
        bridge.run_forever()
    finally:
        right_cam.close()
        if top_cam is not None:
            top_cam.close()
        robot_client.close()
        ws_client.close()
