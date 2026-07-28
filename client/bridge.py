from __future__ import annotations

import argparse
import logging
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from openpi_ws_client import OpenPIWebsocketClient
from robot_tcp_client import RobotTcpClient
from usb_camera import CameraConfig, USBCamera

logger = logging.getLogger(__name__)


def _normalize_state_7d(values: list[float]) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float32).reshape(-1)
    if arr.shape[0] >= 7:
        out = arr[:7]
    else:
        out = np.concatenate([arr, np.zeros((7 - arr.shape[0],), dtype=np.float32)], axis=0)
    return np.ascontiguousarray(out.reshape(1, 7), dtype=np.float32)


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
        right_rgb = self._right_cam.read_rgb()
        if self._top_cam is not None:
            top_rgb = self._top_cam.read_rgb()
        else:
            top_rgb = right_rgb

        right_chw = USBCamera.preprocess_to_policy_chw(right_rgb, size=224)
        top_chw = USBCamera.preprocess_to_policy_chw(top_rgb, size=224)

        return {
            "observation/state": state_7d,
            "observation/images/right": right_chw,
            "observation/images/top": top_chw,
            "prompt": self._cfg.prompt,
        }

    def run_forever(self) -> None:
        """推理 → ``SET_JOINTS_BATCH`` → 等机器人逐步执行完 → 再采 obs / 下一轮推理。"""
        infer_period_s = 1.0 / self._cfg.hz if self._cfg.hz > 0 else 0.0
        infer_cycle = 0
        if self._cfg.start_delay_s > 0:
            logger.info("启动延时 %.3fs：延时结束后开始发送 obs / 推理 / 执行动作。", self._cfg.start_delay_s)
            time.sleep(self._cfg.start_delay_s)
        while True:
            t_cycle = time.time()
            try:
                robot_state = self._robot.get_follower_state()
                state_7d = _normalize_state_7d(robot_state)
                obs = self._build_observation(state_7d)
                infer_result = self._ws.infer(obs)
                action_chunk = _extract_action_chunk(infer_result, expected_dim=self._cfg.action_dim)
                n = int(action_chunk.shape[0])
                if n <= 0:
                    logger.warning("推理返回的 action 序列为空，跳过本轮。")
                    time.sleep(0.05)
                    continue

                self._robot.send_policy_actions_batch(action_chunk)
                self._robot.wait_policy_idle(
                    poll_s=self._cfg.idle_poll_s,
                    timeout_s=self._cfg.idle_timeout_s,
                )
                if self._cfg.post_idle_settle_s > 0:
                    # 机器人端 robot_q_ 由独立线程刷新，batch 期间会冻结；idle 后给一次刷新窗口，
                    # 确保下一轮观测更接近“上一段动作执行结束”的真实位置。
                    time.sleep(self._cfg.post_idle_settle_s)

                if infer_cycle % 20 == 0:
                    logger.info(
                        "infer_cycle=%d chunk_len=%d state=%s action[0]=%s",
                        infer_cycle,
                        n,
                        np.array2string(state_7d[0], precision=4, suppress_small=True),
                        np.array2string(action_chunk[0], precision=4, suppress_small=True),
                    )
                infer_cycle += 1
            except Exception:  # noqa: BLE001
                logger.exception("Bridge step failed. Retrying next cycle.")
                time.sleep(0.2)

            if infer_period_s > 0:
                elapsed = time.time() - t_cycle
                remain = infer_period_s - elapsed
                if remain > 0:
                    time.sleep(remain)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Bridge OpenPI websocket inference with this repo's a10_tcp_server (TCP).")
    parser.add_argument("--policy-host", type=str, default="127.0.0.1")
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
        help="顶视 / third-person 相机（默认 video0）。传空字符串则复用右相机画面。",
    )
    parser.add_argument(
        "--right-rotate-180",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="右相机画面旋转 180° 再送入策略（倒装安装时开启，默认关闭）。",
    )
    parser.add_argument(
        "--top-rotate-180",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="顶视相机画面旋转 180° 再送入策略（倒装安装时开启，默认开启）。",
    )
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--prompt", type=str, default="Reach the target fruit on the table.")
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
