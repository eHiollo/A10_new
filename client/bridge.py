from __future__ import annotations

import argparse
import collections
import logging
from pathlib import Path
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from openpi_ws_client import OpenPIWebsocketClient
from robot_tcp_client import RobotTcpClient
from temporal_ensemble import TemporalEnsemble
from usb_camera import CameraConfig, ThreadedCamera, USBCamera

logger = logging.getLogger(__name__)


def _state_7d(values: list[float]) -> np.ndarray:
    """``(7,) float32``：关节 0–5 为绝对角（rad），第 6 维为夹爪绝对开度（mm）。

    不做归一化，quantile 统计在推理服务端。
    """
    arr = np.asarray(values, dtype=np.float32).reshape(-1)
    if arr.shape != (7,) or not np.isfinite(arr).all():
        raise ValueError("Observation state must contain seven finite values")
    return np.ascontiguousarray(arr, dtype=np.float32)


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
    if actions.shape[0] == 0 or not np.isfinite(actions).all():
        raise ValueError("Policy returned empty or non-finite actions")

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
    mode: str = "async"
    buffer_target: int = 1
    infer_backoff_s: float = 0.005
    prefetch_lead_s: float = 0.45
    async_smoothing: bool = True
    temporal_ensemble: bool = True
    ensemble_decay: float = 0.01
    ensemble_gripper: bool = True


class PiRobotBridge:
    def __init__(
        self,
        ws_client: OpenPIWebsocketClient,
        robot_client: RobotTcpClient,
        right_cam: USBCamera | ThreadedCamera,
        top_cam: USBCamera | ThreadedCamera | None,
        cfg: BridgeConfig,
        runtime_log=None,
    ) -> None:
        self._ws = ws_client
        self._robot = robot_client
        self._right_cam = right_cam
        self._top_cam = top_cam
        self._cfg = cfg
        self._log = runtime_log
        self._infer_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._async_buffer: collections.deque[tuple[np.ndarray, float]] = collections.deque()
        self._async_buffer_lock = threading.Lock()
        self._async_buffer_not_empty = threading.Condition(self._async_buffer_lock)

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

    def _obs_state_7d(self) -> np.ndarray:
        return _state_7d(self._robot.get_follower_state())

    def _infer_chunk(self, state_7d: np.ndarray) -> np.ndarray:
        log_session = getattr(self._log, "session_id", None)
        started = time.monotonic()
        obs = self._build_observation(state_7d)
        observed = time.monotonic()
        if self._log is not None:
            self._log.save_observation_images(obs)
        infer_started = time.monotonic()
        infer_result = self._ws.infer(obs)
        self._last_infer_result = infer_result
        chunk = _extract_action_chunk(infer_result, expected_dim=self._cfg.action_dim)
        if self._log is not None:
            emit = self._log.write
            if hasattr(self._log, "write_for_session"):
                emit = lambda event, **fields: self._log.write_for_session(log_session, event, **fields)
            emit("inference", state=state_7d, state_info=self._robot.last_state_info,
                            cameras={"right": self._right_cam.last_frame_info,
                                     "top": self._top_cam.last_frame_info if self._top_cam is not None else None},
                            observation_ms=(observed - started) * 1000.0,
                            inference_roundtrip_ms=(time.monotonic() - infer_started) * 1000.0,
                            raw_action_shape=list(np.shape(infer_result["actions"])), actions=chunk,
                            policy_timing=infer_result.get("policy_timing"),
                            server_timing=infer_result.get("server_timing"),
                            transport_timing=getattr(self._ws, "last_transport_timing", {}),
                            action_delta_from_state=chunk[0] - state_7d)
        return chunk

    def stop(self) -> None:
        self._stop.set()
        with self._async_buffer_lock:
            self._async_buffer_not_empty.notify_all()

    def run_forever(self) -> None:
        def monitor() -> None:
            previous_running = None
            while not self._stop.is_set():
                try:
                    status = self._robot.get_policy_status()
                    running = status.get("policy_running")
                    if running is not None and running != previous_running:
                        logger.info("policy %s；bridge 保持运行。", "正在运行，允许下发新动作" if running else "未运行，继续取图/推理，暂不下发动作")
                        if self._log is not None:
                            self._log.write("policy_state_change", running=running)
                        previous_running = running
                except Exception as exc:
                    if self._log is not None:
                        self._log.write("status_error", error=repr(exc))
                self._stop.wait(0.1)

        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
        if self._cfg.mode == "async":
            logger.info("async 按推理完成速度换段；--hz 仅作用于 sync，不改变动作时间步。")
        try:
            if self._cfg.mode == "async":
                self._run_async()
            elif self._cfg.mode == "pipeline":
                self._run_pipeline()
            else:
                self._run_sync()
        finally:
            self.stop()
            monitor_thread.join(timeout=2.0)

    def _run_sync(self) -> None:
        """推理 → ``SET_JOINTS_BATCH`` → 等机器人逐步执行完 → 再采 obs / 下一轮推理。"""
        infer_period_s = 1.0 / self._cfg.hz if self._cfg.hz > 0 else 0.0
        infer_cycle = 0
        if self._cfg.start_delay_s > 0:
            logger.info("启动延时 %.3fs：延时结束后开始发送 obs / 推理 / 执行动作。", self._cfg.start_delay_s)
            time.sleep(self._cfg.start_delay_s)
        while not self._stop.is_set():
            t_cycle = time.time()
            try:
                t_obs = time.perf_counter()
                state_7d = self._obs_state_7d()
                action_chunk = self._infer_chunk(state_7d)
                n = int(action_chunk.shape[0])
                if n <= 0:
                    logger.warning("推理返回的 action 序列为空，跳过本轮。")
                    time.sleep(0.05)
                    continue

                inference_elapsed_s = time.perf_counter() - t_obs
                if self._robot.get_policy_status().get("policy_running") is False:
                    self._stop.wait(0.1)
                    continue
                # sync 模式下，推理期间机器人保持在上一批终点，动作时间轴尚未开始。
                # 不把推理耗时作为轨迹年龄，否则 0.38s 会错误裁掉约 11 个 30Hz 动作。
                queued = self._robot.send_policy_actions_batch(action_chunk, obs_age_s=None)
                if self._log is not None:
                    self._log.write("sync_send", cycle=infer_cycle, queued=queued,
                                    inference_elapsed_s=inference_elapsed_s,
                                    obs_age_s_sent=None, ack=self._robot.last_batch_ack)
                wait_st = self._robot.wait_policy_idle(
                    poll_s=self._cfg.idle_poll_s,
                    timeout_s=self._cfg.idle_timeout_s,
                    should_stop=self._stop.is_set,
                    expected_queued=queued,
                )
                if wait_st == "skipped":
                    logger.warning("sync: batch 被丢弃或 policy 已停止，bridge 保持运行。")
                    self._stop.wait(0.2)
                    continue
                if wait_st == "stopped" or self._stop.is_set():
                    break
                if self._cfg.post_idle_settle_s > 0:
                    time.sleep(self._cfg.post_idle_settle_s)

                if infer_cycle % 20 == 0:
                    policy_timing = getattr(self, "_last_infer_result", {}).get("policy_timing") or {}
                    logger.info(
                        "sync infer_cycle=%d chunk_len=%d infer_ms=%s state=%s action[0]=%s",
                        infer_cycle,
                        n,
                        policy_timing.get("infer_ms"),
                        np.array2string(state_7d, precision=4, suppress_small=True),
                        np.array2string(action_chunk[0], precision=4, suppress_small=True),
                    )
                infer_cycle += 1
            except Exception as exc:
                logger.exception("Bridge sync step failed; will retry.")
                if self._log is not None:
                    self._log.write("inference_error", error=repr(exc))
                self._stop.wait(0.5)

            if infer_period_s > 0:
                elapsed = time.time() - t_cycle
                remain = infer_period_s - elapsed
                if remain > 0:
                    time.sleep(remain)

    def _wait_pipeline_window(self, lead_s: float) -> dict | None:
        while not self._stop.is_set():
            status = self._robot.get_policy_status()
            if status.get("policy_running") is False or int(status.get("remaining", 0)) == 0:
                return status
            remaining_s = status.get("remaining_time_s", -1.0)
            if remaining_s >= 0.0 and remaining_s <= lead_s:
                return status
            if "remaining_time_s" not in status and not getattr(self, "_pipeline_fallback_warned", False):
                logger.warning("控制器缺少剩余执行时间，pipeline 暂按 sync 等待；重启新编译的控制器后可提前推理。")
                self._pipeline_fallback_warned = True
            self._stop.wait(max(0.01, self._cfg.idle_poll_s))
        return None

    def _wait_pipeline_boundary(self, expected_seq: int, deadline: float) -> dict | None:
        while not self._stop.is_set():
            status = self._robot.get_policy_status()
            reason = None
            if status.get("policy_running") is False:
                reason = "policy_stopped"
            elif int(status.get("batch_seq", 0)) != expected_seq:
                reason = "batch_changed"
            elif time.perf_counter() >= deadline:
                reason = "prediction_expired"
            if reason is not None:
                if self._log is not None:
                    self._log.write("pipeline_discard", reason=reason, expected_batch_seq=expected_seq, status=status)
                return None
            if int(status.get("remaining", 0)) == 0 and not status.get("motion_settling", False):
                return status
            self._stop.wait(max(0.005, self._cfg.idle_poll_s))
        return None

    def _run_pipeline(self) -> None:
        latencies: collections.deque[float] = collections.deque(maxlen=20)
        horizon_s = 1.0
        logger.info("pipeline：尾段预取，旧 batch 完成后衔接；初始提前量 %.3fs。", self._cfg.prefetch_lead_s)
        self._stop.wait(self._cfg.start_delay_s)
        while not self._stop.is_set():
            try:
                measured_lead = float(np.percentile(latencies, 95)) + 0.05 if latencies else 0.0
                lead_s = min(horizon_s * 0.8, max(self._cfg.prefetch_lead_s, measured_lead))
                before = self._wait_pipeline_window(lead_s)
                if before is None:
                    break
                expected_seq = int(before.get("batch_seq", 0))
                if self._log is not None:
                    self._log.write("pipeline_prefetch", lead_s=lead_s, status=before)
                observed_at = time.perf_counter()
                state = self._obs_state_7d()
                chunk = self._infer_chunk(state)
                inferred_at = time.perf_counter()
                horizon_s = len(chunk) * float(before.get("action_step_s", 1.0 / 30.0))
                inference_s = inferred_at - observed_at
                if inference_s < horizon_s:
                    latencies.append(inference_s)
                if before.get("policy_running") is False:
                    if self._log is not None:
                        self._log.write("idle_prediction", chunk_len=len(chunk), obs_age_s=inference_s)
                    continue
                boundary = self._wait_pipeline_boundary(expected_seq, observed_at + horizon_s)
                if boundary is None or self._stop.is_set():
                    continue
                obs_age_s = time.perf_counter() - observed_at
                queued = self._robot.send_policy_actions_batch(chunk, obs_age_s=obs_age_s, expected_batch_seq=expected_seq)
                if self._log is not None:
                    self._log.write("pipeline_send", queued=queued, obs_age_s=obs_age_s,
                                    remaining_before_inference=before.get("remaining"),
                                    estimated_remaining_s=before.get("remaining_time_s"),
                                    wait_for_boundary_s=time.perf_counter() - inferred_at,
                                    ack=self._robot.last_batch_ack)
            except Exception as exc:
                logger.exception("pipeline step failed; will retry.")
                if self._log is not None:
                    self._log.write("pipeline_error", error=repr(exc))
                self._stop.wait(0.5)

    def _run_async(self) -> None:
        """执行中采观测并推理，下一段到达后替换还没走完的队列。

        控制器从当前指令姿态接到新队首，不等 remaining==0。
        """
        buffer = self._async_buffer
        buffer_lock = self._async_buffer_lock
        buffer_not_empty = self._async_buffer_not_empty
        infer_cycle = 0
        ensemble = TemporalEnsemble(self._cfg.ensemble_decay, self._cfg.ensemble_gripper)
        ensemble_sequence = None

        if self._cfg.start_delay_s > 0:
            logger.info("async 启动延时 %.3fs。", self._cfg.start_delay_s)
            time.sleep(self._cfg.start_delay_s)

        def inferencer() -> None:
            while not self._stop.is_set():
                try:
                    with buffer_lock:
                        if len(buffer) >= self._cfg.buffer_target:
                            buffer_not_empty.wait(timeout=self._cfg.infer_backoff_s)
                            continue
                    t_obs = time.perf_counter()
                    state_7d = self._obs_state_7d()
                    chunk = self._infer_chunk(state_7d)
                    with buffer_lock:
                        if self._stop.is_set():
                            return
                        buffer.append((chunk, t_obs))
                        buffer_not_empty.notify()
                except Exception as exc:
                    logger.exception("async inferencer step failed.")
                    if self._log is not None:
                        self._log.write("inference_error", error=repr(exc))
                    self._stop.wait(0.5)

        infer_thread = threading.Thread(target=inferencer, daemon=True)
        self._infer_thread = infer_thread
        infer_thread.start()
        with buffer_lock:
            while len(buffer) == 0 and not self._stop.is_set():
                buffer_not_empty.wait(timeout=1.0)
        if self._stop.is_set():
            return

        while not self._stop.is_set():
            try:
                with buffer_lock:
                    while len(buffer) == 0 and not self._stop.is_set():
                        buffer_not_empty.wait(timeout=1.0)
                    if self._stop.is_set():
                        break
                    if len(buffer) == 0:
                        continue
                    action_chunk, t_obs = buffer.popleft()
                    buffer_not_empty.notify_all()

                n = int(action_chunk.shape[0])
                if n <= 0:
                    logger.warning("async: chunk 为空，跳过。")
                    continue

                status = self._robot.get_policy_status()
                if status.get("policy_running") is False:
                    ensemble.clear()
                    ensemble_sequence = None
                    if self._log is not None:
                        self._log.write("idle_prediction", chunk_len=n, obs_age_s=time.perf_counter() - t_obs)
                    continue
                if self._cfg.temporal_ensemble:
                    if status.get("batch_seq") != ensemble_sequence:
                        ensemble.clear()
                    raw_actions = action_chunk
                    action_chunk, diagnostics = ensemble.fuse(raw_actions, t_obs, float(status.get("action_step_s", 1.0 / 30.0)))
                    if self._log is not None:
                        self._log.write("temporal_ensemble", observed_at=t_obs, raw_actions=raw_actions,
                                        actions=action_chunk, **diagnostics)
                rem_before = int(status.get("remaining") or 0)
                obs_age_s = time.perf_counter() - t_obs
                smooth = self._cfg.async_smoothing and status.get("supports_async_smoothing") is True
                if self._cfg.async_smoothing and not smooth and not getattr(self, "_smoothing_warned", False):
                    logger.warning("控制器不支持异步平滑，暂按原逻辑执行；停止机器人并重启新编译的控制器后生效。")
                    self._smoothing_warned = True
                queued = self._robot.send_policy_actions_batch(action_chunk, obs_age_s=obs_age_s, smooth=smooth)
                ensemble_sequence = self._robot.last_batch_ack.get("batch_seq")
                if self._log is not None:
                    self._log.write("async_send", cycle=infer_cycle, remaining_before=rem_before,
                                    queued=queued, obs_age_s=obs_age_s, smooth=smooth, ack=self._robot.last_batch_ack)
                if infer_cycle > 0 and rem_before <= 0:
                    logger.warning(
                        "async: cycle=%d rem_before=%s 队列已空才发下一段（没有叠上执行）",
                        infer_cycle,
                        rem_before,
                    )
                if infer_cycle % 20 == 0:
                    policy_timing = getattr(self, "_last_infer_result", {}).get("policy_timing") or {}
                    logger.info(
                        "async send cycle=%d chunk_len=%d rem_before=%s queued=%d obs_age_s=%.3f infer_ms=%s action[0]=%s",
                        infer_cycle,
                        n,
                        rem_before,
                        queued,
                        obs_age_s,
                        policy_timing.get("infer_ms"),
                        np.array2string(action_chunk[0], precision=4, suppress_small=True),
                    )
                infer_cycle += 1
            except Exception as exc:
                logger.exception("async pusher step failed.")
                ensemble.clear()
                ensemble_sequence = None
                if self._log is not None:
                    self._log.write("transport_error", error=repr(exc))
                self._stop.wait(0.5)


class BridgeArgumentParser(argparse.ArgumentParser):
    def parse_args(self, args=None, namespace=None):
        parsed = super().parse_args(args, namespace)
        if parsed.temporal_ensemble is None:
            parsed.temporal_ensemble = parsed.mode == "async"
        return parsed


def build_arg_parser() -> argparse.ArgumentParser:
    parser = BridgeArgumentParser(allow_abbrev=False, description="Bridge OpenPI websocket inference with this repo's a10_tcp_server (TCP).")
    parser.add_argument("--log-dir", default=str(Path(__file__).resolve().parent / "logs"), help="日志根目录；由控制器 policy --log=1/0 控制，每次 policy 新建独立目录。")
    parser.add_argument("--camera-max-age", type=float, default=0.5, help="仅日志标记：相机缓存帧过旧阈值（秒），不阻止 bridge 运行。")
    parser.add_argument("--infer-timeout", type=float, default=60.0, help="推理回包超时（秒），需包含首次预热。")
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
        default="0",
        help="右腕 / wrist 相机（公司实机已确认默认 video0，即索引 0 或 /dev/video0）。",
    )
    parser.add_argument(
        "--top-camera",
        type=str,
        default="2",
        help="顶视 / third-person 相机（公司实机已确认默认 video2），对应 observation/images/top。传空字符串则不发这一路，服务端按没看到处理。",
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
        help="仅 sync：整轮最小周期为 1/hz 秒；async 不使用此参数，也不改变控制器的动作时间步。",
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
    parser.add_argument(
        "--mode",
        choices=["async", "sync", "pipeline"],
        default="async",
        help="async=执行中不断替换；sync=走完再推理；pipeline=尾段提前推理、走完再衔接。",
    )
    parser.add_argument("--prefetch-lead", type=float, default=0.45, help="pipeline 提前推理的初始秒数，会按实测推理耗时调整。")
    parser.add_argument("--temporal-ensemble", action=argparse.BooleanOptionalAction, default=None,
                        help="async 默认开启 ACT 式指数融合；--no-temporal-ensemble 关闭。sync/pipeline 默认关闭。")
    parser.add_argument("--ensemble-decay", type=float, default=0.01, help="指数权重衰减，旧预测在前；0 为平均。")
    parser.add_argument("--ensemble-gripper", action=argparse.BooleanOptionalAction, default=True,
                        help="是否融合夹爪；--no-ensemble-gripper 仅融合手臂。")
    parser.add_argument("--async-smoothing", action=argparse.BooleanOptionalAction, default=True,
                        help="async 使用控制器加速度受限衔接；--no-async-smoothing 恢复原插值供对照。")
    parser.add_argument(
        "--camera-mode",
        choices=["threaded", "exclusive"],
        default="threaded",
        help="threaded=后台持续抓最新帧；exclusive=每帧独占打开（同总线掉帧时用）。",
    )
    parser.add_argument(
        "--buffer-target",
        type=int,
        default=1,
        help="async 模式下预先缓冲的 chunk 数，1 表示双缓冲。",
    )
    return parser


def _make_camera(source: str, width: int, height: int, rotate_180: bool, camera_mode: str, exclusive: bool, max_age_s: float = 0.5):
    cfg = CameraConfig(
        source=source,
        width=width,
        height=height,
        rotate_180=rotate_180,
        exclusive_per_read=exclusive,
        max_frame_age_s=max_age_s,
    )
    if camera_mode == "threaded":
        return ThreadedCamera(cfg)
    return USBCamera(cfg)


def run_from_args(args: argparse.Namespace, runtime_log=None) -> None:
    if args.action_dim != 7:
        raise ValueError("This robot requires --action-dim 7")
    if not np.isfinite(args.camera_max_age) or args.camera_max_age <= 0:
        raise ValueError("--camera-max-age must be positive and finite")
    if not np.isfinite(args.infer_timeout) or args.infer_timeout <= 0:
        raise ValueError("--infer-timeout must be positive and finite")
    if not np.isfinite(args.prefetch_lead) or args.prefetch_lead <= 0:
        raise ValueError("--prefetch-lead must be positive and finite")
    if not np.isfinite(args.ensemble_decay) or args.ensemble_decay < 0:
        raise ValueError("--ensemble-decay must be finite and nonnegative")
    if args.temporal_ensemble and args.mode != "async":
        raise ValueError("--temporal-ensemble only supports --mode async")
    ws_client = OpenPIWebsocketClient(
        host=args.policy_host,
        port=args.policy_port,
        api_key=args.policy_api_key,
        infer_timeout_s=args.infer_timeout,
    )
    robot_client = RobotTcpClient(host=args.robot_host, port=args.robot_port, runtime_log=runtime_log)
    robot_client.connect()
    if runtime_log is not None:
        runtime_log.write("policy_metadata", metadata=ws_client.server_metadata)

    use_dual_cam = isinstance(args.top_camera, str) and args.top_camera.strip()
    exclusive = bool(use_dual_cam) and args.camera_mode == "exclusive"
    right_cam = _make_camera(
        args.right_camera,
        args.camera_width,
        args.camera_height,
        bool(args.right_rotate_180),
        args.camera_mode,
        exclusive,
        args.camera_max_age,
    )
    top_cam = None
    if use_dual_cam:
        top_cam = _make_camera(
            args.top_camera.strip(),
            args.camera_width,
            args.camera_height,
            bool(args.top_rotate_180),
            args.camera_mode,
            exclusive,
            args.camera_max_age,
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
            mode=str(args.mode),
            buffer_target=max(1, int(args.buffer_target)),
            async_smoothing=bool(args.async_smoothing),
            temporal_ensemble=bool(args.temporal_ensemble),
            ensemble_decay=float(args.ensemble_decay),
            ensemble_gripper=bool(args.ensemble_gripper),
            prefetch_lead_s=args.prefetch_lead,
        ),
        runtime_log=runtime_log,
    )

    def _shutdown() -> None:
        bridge.stop()
        logger.info("正在停止桥接：请求机器人 STOP_POLICY 并关闭连接...")
        try:
            robot_client.stop_policy()
            robot_client.get_policy_status()
        except Exception:
            logger.exception("STOP_POLICY 或最终状态查询失败，请使用控制器 stop / 现场急停。")
            if runtime_log is not None:
                runtime_log.write("stop_failed")
        ws_client.close()
        if bridge._infer_thread is not None:
            bridge._infer_thread.join(timeout=2.0)
        right_cam.close()
        if top_cam is not None:
            top_cam.close()
        robot_client.close()

    try:
        bridge.run_forever()
    except KeyboardInterrupt:
        logger.info("收到 Ctrl+C，开始优雅停止。")
    finally:
        _shutdown()
