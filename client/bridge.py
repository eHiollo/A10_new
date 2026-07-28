from __future__ import annotations

import argparse
import collections
import logging
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

# 始终把本文件所在目录加入 sys.path，使同级模块按平铺名导入可用，
# 兼容 ``python -m client.run_bridge`` 与直接运行脚本两种方式。
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from openpi_ws_client import OpenPIWebsocketClient
from robot_tcp_client import RobotTcpClient
from usb_camera import CameraConfig, USBCamera
from verifier import GeometricMedoidVerifier
from candidate_logger import CandidateLogger

logger = logging.getLogger(__name__)


def _normalize_state_7d(values: list[float]) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float32).reshape(-1)
    if arr.shape[0] >= 7:
        out = arr[:7]
    else:
        out = np.concatenate([arr, np.zeros((7 - arr.shape[0],), dtype=np.float32)], axis=0)
    return np.ascontiguousarray(out.reshape(1, 7), dtype=np.float32)


def _extract_action_candidates(result: dict[str, Any], expected_dim: int = 7) -> np.ndarray:
    """从推理响应中抽取候选集，统一为 ``(N, T, D)``（N>1 为 AsyncVLA 批量采样）。"""
    if "actions" not in result:
        raise KeyError(f"OpenPI response missing 'actions'. keys={list(result.keys())}")

    actions = np.asarray(result["actions"], dtype=np.float32)
    if actions.ndim == 1:
        actions = actions[None, None, :]
    elif actions.ndim == 2:
        actions = actions[None, :, :]
    elif actions.ndim != 3:
        raise ValueError(f"Unexpected action shape: {actions.shape}")

    if actions.shape[-1] < expected_dim:
        raise ValueError(f"Action dim too small: {actions.shape[-1]}, expected >= {expected_dim}")

    if actions.shape[-1] > expected_dim:
        actions = actions[..., :expected_dim]

    return np.ascontiguousarray(actions, dtype=np.float32)


def _extract_action_chunk(result: dict[str, Any], expected_dim: int = 7) -> np.ndarray:
    """单次采样路径：要求响应恰含 1 个 chunk，返回 ``(T, D)``。"""
    candidates = _extract_action_candidates(result, expected_dim)
    if candidates.shape[0] != 1:
        raise ValueError(f"Unexpected action shape: expected single chunk, got {candidates.shape}")
    return candidates[0]


@dataclass
class BridgeConfig:
    prompt: str
    hz: float
    action_dim: int
    idle_poll_s: float
    idle_timeout_s: float
    start_delay_s: float
    post_idle_settle_s: float
    mode: str = "async"  # "async" | "sync"
    # async 模式专用：推理与执行重叠时，buffer 期望缓冲的 chunk 数（双缓冲=1）。
    buffer_target: int = 1
    # async 模式专用：buffer 已满时 Inferencer 的轮询间隔。
    infer_backoff_s: float = 0.005
    # AsyncVLA：每次推理采样的候选 chunk 数（1=原单次采样行为）。
    sample_n: int = 1
    # AsyncVLA verifier：夹爪维度权重（夹爪为 mm 绝对位置，量级远大于关节 rad，
    # 不加权会主导候选间距离）。
    verifier_gripper_weight: float = 0.03
    # AsyncVLA：候选记录目录（None=不记录）；pilot 时开启以分析 divergence
    # 分布与维度构成（校准夹爪权重）。
    log_candidates_dir: str | None = None
    # AsyncVLA adaptive N：开启后忽略 sample_n，按 divergence EMA 在
    # {1,2,4,8} 档位间切换（离散档位 + 滞后降档，避免频繁触发服务端
    # jit 重编译）。阈值单位与 verifier divergence 一致，pilot 后用
    # analyze_candidates.py 的 p50/p90 校准。
    adaptive_n: bool = False
    div_low: float = 0.05
    div_mid: float = 0.15
    div_high: float = 0.30
    # 降档滞后：目标档位连续低于当前档位该轮数后才真正降档。
    downshift_patience: int = 5
    # N=1 时无 divergence 信号（单次采样路径不算 verifier），
    # 每隔该轮数用 4 候选探测一次分歧，防止"降档后遇复杂动作升不回来"。
    probe_interval: int = 20


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
        self._stop = threading.Event()
        # async 模式的 action buffer 与条件变量(实例化以便 stop() 能唤醒等待者)。
        self._async_buffer: collections.deque[np.ndarray] = collections.deque()
        self._async_buffer_lock = threading.Lock()
        self._async_buffer_not_empty = threading.Condition(self._async_buffer_lock)
        # AsyncVLA L1 verifier：夹爪维度（最后一维）降权，其余维度等权。
        dim_weights = np.ones(cfg.action_dim, dtype=np.float64)
        dim_weights[-1] = cfg.verifier_gripper_weight
        self._verifier = GeometricMedoidVerifier(dim_weights=dim_weights)
        self._infer_count = 0
        self._cand_logger: CandidateLogger | None = None
        if cfg.log_candidates_dir:
            self._cand_logger = CandidateLogger(cfg.log_candidates_dir)
        # adaptive N 状态：中间档起步, EMA 未初始化。
        self._current_n = 4
        self._div_ema: float | None = None
        self._below_count = 0
        self._probe_count = 0

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

    def _decide_sample_n(self) -> int:
        """本轮采样数：固定模式返回 sample_n；adaptive 模式返回当前档位。

        N=1 档位走单次采样路径（无 verifier 即无 divergence），按
        probe_interval 周期插一轮 4 候选探测分歧，保证复杂动作能及时升档。
        """
        if not self._cfg.adaptive_n:
            return self._cfg.sample_n
        if self._current_n == 1:
            self._probe_count += 1
            if self._probe_count >= self._cfg.probe_interval:
                self._probe_count = 0
                return 4
            return 1
        self._probe_count = 0
        return self._current_n

    def _update_adaptive_n(self, divergence: float) -> None:
        """divergence EMA → 目标档位 {1,2,4,8}；升档立即、降档滞后。

        升档判定用 max(ema, 瞬时值)：复杂动作（分歧 spike）当轮即可升档，
        不被 EMA 平滑拖慢；降档仍走 EMA + 滞后，防止噪声抖动与频繁
        jit 重编译。
        """
        alpha = 0.3
        self._div_ema = divergence if self._div_ema is None else (1 - alpha) * self._div_ema + alpha * divergence
        ema = self._div_ema
        cfg = self._cfg

        def _level(signal: float) -> int:
            if signal < cfg.div_low:
                return 1
            if signal < cfg.div_mid:
                return 2
            if signal < cfg.div_high:
                return 4
            return 8

        up_target = _level(max(ema, divergence))
        if up_target > self._current_n:
            logger.info("adaptive-n: 升档 %d → %d (ema=%.4f, 瞬时=%.4f)", self._current_n, up_target, ema, divergence)
            self._current_n = up_target
            self._below_count = 0
            return

        target = _level(ema)
        if target < self._current_n:
            self._below_count += 1
            if self._below_count >= cfg.downshift_patience:
                logger.info("adaptive-n: 降档 %d → %d (ema=%.4f, 滞后%d轮)", self._current_n, target, ema, self._below_count)
                self._current_n = target
                self._below_count = 0
        else:
            self._below_count = 0

    def _infer_chunk(self, state_7d: np.ndarray) -> np.ndarray:
        """采集观测并推理，返回 ``(T, action_dim)`` action chunk。

        采样数 >1 时走 AsyncVLA 路径：服务端单次 batch 采样 N 个候选，
        本地 L1 verifier（几何一致性 medoid）选优并记录分歧度。
        """
        obs = self._build_observation(state_7d)
        n = self._decide_sample_n()
        if n <= 1:
            result = self._ws.infer(obs)
            return _extract_action_chunk(result, expected_dim=self._cfg.action_dim)

        result = self._ws.infer(obs, sample_n=n)
        candidates = _extract_action_candidates(result, expected_dim=self._cfg.action_dim)
        vr = self._verifier.select(candidates)
        self._infer_count += 1
        if self._cand_logger is not None:
            try:
                self._cand_logger.log(self._infer_count, candidates, vr.best_index, vr.divergence, vr.per_candidate_mean_dist)
            except Exception:  # noqa: BLE001
                logger.exception("候选记录写入失败（不影响主流程）。")
        if self._cfg.adaptive_n:
            self._update_adaptive_n(vr.divergence)
        if self._infer_count % 20 == 0:
            logger.info(
                "verifier: N=%d best=%d divergence=%.4f per_cand_mean=%s",
                candidates.shape[0],
                vr.best_index,
                vr.divergence,
                np.array2string(vr.per_candidate_mean_dist, precision=4, suppress_small=True),
            )
        return vr.best

    def run_forever(self) -> None:
        if self._cfg.mode == "async":
            self._run_async()
        else:
            self._run_sync()

    def _run_sync(self) -> None:
        """同步模式：推理 → SET_JOINTS_BATCH → 等执行完 → 再采 obs / 下一轮推理。"""
        infer_period_s = 1.0 / self._cfg.hz if self._cfg.hz > 0 else 0.0
        infer_cycle = 0
        if self._cfg.start_delay_s > 0:
            logger.info("启动延时 %.3fs：延时结束后开始发送 obs / 推理 / 执行动作。", self._cfg.start_delay_s)
            time.sleep(self._cfg.start_delay_s)
        while not self._stop.is_set():
            t_cycle = time.time()
            try:
                robot_state = self._robot.get_follower_state()
                state_7d = _normalize_state_7d(robot_state)
                action_chunk = self._infer_chunk(state_7d)
                n = int(action_chunk.shape[0])
                if n <= 0:
                    logger.warning("推理返回的 action 序列为空，跳过本轮。")
                    time.sleep(0.05)
                    continue

                self._robot.send_policy_actions_batch(action_chunk)
                self._robot.wait_policy_idle(
                    poll_s=self._cfg.idle_poll_s,
                    timeout_s=self._cfg.idle_timeout_s,
                    should_stop=self._stop.is_set,
                )
                if self._stop.is_set():
                    break
                if self._cfg.post_idle_settle_s > 0:
                    time.sleep(self._cfg.post_idle_settle_s)

                if infer_cycle % 20 == 0:
                    logger.info(
                        "sync infer_cycle=%d chunk_len=%d state=%s action[0]=%s",
                        infer_cycle,
                        n,
                        np.array2string(state_7d[0], precision=4, suppress_small=True),
                        np.array2string(action_chunk[0], precision=4, suppress_small=True),
                    )
                infer_cycle += 1
            except Exception:  # noqa: BLE001
                logger.exception("Bridge sync step failed. Retrying next cycle.")
                time.sleep(0.2)

            if infer_period_s > 0:
                elapsed = time.time() - t_cycle
                remain = infer_period_s - elapsed
                if remain > 0:
                    time.sleep(remain)

    def _run_async(self) -> None:
        """异步双缓冲模式：推理 chunk_{k+1} 与执行 chunk_k 重叠，消除 batch 间 GPU 空闲间隙。

        时序：
          启动: obs → infer → chunk_0 入 buffer
          稳态每轮:
            chunk = buffer.pop()           # 上一轮已推理好的
            push chunk 到机器人             # 开始执行(耗时 T_exec)
            ── 重叠窗口 ──
            obs = get_follower_state()     # 当前 chunk 开始时的状态(A10 已改为 batch 期间也刷新, 故新鲜)
            next_chunk = infer(obs)         # 后台推理, 与执行重叠
            buffer.append(next_chunk)
            wait_policy_idle()             # 等 chunk 执行完
            ── 下一轮 ──

        观测比同步模式陈旧约 1 个 chunk（open-loop chunk 执行的标准权衡）。
        """
        buffer = self._async_buffer
        buffer_lock = self._async_buffer_lock
        buffer_not_empty = self._async_buffer_not_empty
        infer_cycle = 0

        if self._cfg.start_delay_s > 0:
            logger.info("async 启动延时 %.3fs。", self._cfg.start_delay_s)
            time.sleep(self._cfg.start_delay_s)

        def inferencer() -> None:
            """后台推理线程：buffer 未满时采 obs 推理并入队。"""
            while not self._stop.is_set():
                try:
                    with buffer_lock:
                        if len(buffer) >= self._cfg.buffer_target:
                            buffer_not_empty.wait(timeout=self._cfg.infer_backoff_s)
                            continue
                    robot_state = self._robot.get_follower_state()
                    state_7d = _normalize_state_7d(robot_state)
                    chunk = self._infer_chunk(state_7d)
                    with buffer_lock:
                        buffer.append(chunk)
                        buffer_not_empty.notify()
                except Exception:  # noqa: BLE001
                    logger.exception("async inferencer step failed.")
                    time.sleep(0.2)

        # 启动推理线程并等待首个 chunk 就绪（首轮无重叠）。
        t_inf = threading.Thread(target=inferencer, daemon=True)
        t_inf.start()
        with buffer_lock:
            while len(buffer) == 0 and not self._stop.is_set():
                buffer_not_empty.wait(timeout=1.0)
        if self._stop.is_set():
            return

        # 主循环 = Pusher：取已推理好的 chunk 推送，等执行完，循环。
        while not self._stop.is_set():
            try:
                with buffer_lock:
                    while len(buffer) == 0 and not self._stop.is_set():
                        buffer_not_empty.wait(timeout=1.0)
                    if self._stop.is_set():
                        break
                    action_chunk = buffer.popleft()

                n = int(action_chunk.shape[0])
                if n <= 0:
                    logger.warning("async: chunk 为空，跳过。")
                    continue

                self._robot.send_policy_actions_batch(action_chunk)
                self._robot.wait_policy_idle(
                    poll_s=self._cfg.idle_poll_s,
                    timeout_s=self._cfg.idle_timeout_s,
                    should_stop=self._stop.is_set,
                )
                if self._stop.is_set():
                    break
                if self._cfg.post_idle_settle_s > 0:
                    time.sleep(self._cfg.post_idle_settle_s)

                if infer_cycle % 20 == 0:
                    logger.info(
                        "async infer_cycle=%d chunk_len=%d buffer=%d action[0]=%s",
                        infer_cycle,
                        n,
                        len(buffer),
                        np.array2string(action_chunk[0], precision=4, suppress_small=True),
                    )
                infer_cycle += 1
            except Exception:  # noqa: BLE001
                logger.exception("async pusher step failed.")
                time.sleep(0.2)

    def stop(self) -> None:
        self._stop.set()
        # 唤醒可能阻塞在 buffer 条件变量上的 inferencer / pusher, 让它们立即看到 stop。
        with self._async_buffer_lock:
            self._async_buffer_not_empty.notify_all()
        if self._cand_logger is not None:
            self._cand_logger.close()


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
    parser.add_argument(
        "--mode",
        type=str,
        default="async",
        choices=["async", "sync"],
        help="async=双缓冲流水线(推理与执行重叠, 推荐); sync=原同步模式(作对比/回退)。",
    )
    parser.add_argument(
        "--camera-mode",
        type=str,
        default="threaded",
        choices=["threaded", "exclusive"],
        help="threaded=后台线程持续抓帧(异步推荐, 采图不阻塞); exclusive=每帧独占 open/close(多相机同总线 fallback)。",
    )
    parser.add_argument(
        "--buffer-target",
        type=int,
        default=1,
        help="async 模式: buffer 期望缓冲的 chunk 数(双缓冲=1)。",
    )
    parser.add_argument(
        "--sample-n",
        type=int,
        default=1,
        help="AsyncVLA: 每次推理采样的候选 chunk 数(服务端单次 batch 完成; 1=原单次采样)。",
    )
    parser.add_argument(
        "--verifier-gripper-weight",
        type=float,
        default=0.03,
        help="AsyncVLA verifier: 夹爪维度权重(夹爪 mm 量级远大于关节 rad, 降权避免主导距离)。",
    )
    parser.add_argument(
        "--log-candidates",
        type=str,
        default=None,
        help="AsyncVLA: 候选记录目录(jsonl, 含全量候选; pilot 分析用)。不传则不记录。",
    )
    parser.add_argument(
        "--adaptive-n",
        action="store_true",
        help="AsyncVLA: 按 divergence EMA 在 {1,2,4,8} 档位自适应采样数(忽略 --sample-n; 阈值 pilot 后校准)。",
    )
    parser.add_argument("--div-low", type=float, default=0.05, help="adaptive-n 阈值: 低于此 divergence EMA 降到 N=1。")
    parser.add_argument("--div-mid", type=float, default=0.15, help="adaptive-n 阈值: N=2/N=4 分界。")
    parser.add_argument("--div-high", type=float, default=0.30, help="adaptive-n 阈值: 高于此升到 N=8。")
    parser.add_argument("--downshift-patience", type=int, default=5, help="adaptive-n 降档滞后轮数(防抖动+防频繁 jit 重编译)。")
    parser.add_argument("--probe-interval", type=int, default=20, help="adaptive-n N=1 档位时每 N 轮插一轮 4 候选分歧探测。")
    return parser


def _make_camera(source: str, width: int, height: int, rotate_180: bool, camera_mode: str, exclusive: bool):
    cfg = CameraConfig(
        source=source,
        width=width,
        height=height,
        rotate_180=rotate_180,
        exclusive_per_read=exclusive,
    )
    if camera_mode == "threaded":
        from usb_camera import ThreadedCamera
        return ThreadedCamera(cfg)
    return USBCamera(cfg)


def run_from_args(args: argparse.Namespace) -> None:
    ws_client = OpenPIWebsocketClient(
        host=args.policy_host,
        port=args.policy_port,
        api_key=args.policy_api_key,
    )
    robot_client = RobotTcpClient(host=args.robot_host, port=args.robot_port)
    robot_client.connect()

    use_dual_cam = isinstance(args.top_camera, str) and args.top_camera.strip()
    # exclusive 模式下双相机同总线需独占；threaded 模式各自持久句柄。
    exclusive = bool(use_dual_cam) and args.camera_mode == "exclusive"

    right_cam = _make_camera(args.right_camera, args.camera_width, args.camera_height, bool(args.right_rotate_180), args.camera_mode, exclusive)
    top_cam = None
    if use_dual_cam:
        top_cam = _make_camera(args.top_camera.strip(), args.camera_width, args.camera_height, bool(args.top_rotate_180), args.camera_mode, exclusive)

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
            sample_n=max(1, int(args.sample_n)),
            verifier_gripper_weight=float(args.verifier_gripper_weight),
            log_candidates_dir=args.log_candidates if args.log_candidates else None,
            adaptive_n=bool(args.adaptive_n),
            div_low=float(args.div_low),
            div_mid=float(args.div_mid),
            div_high=float(args.div_high),
            downshift_patience=max(1, int(args.downshift_patience)),
            probe_interval=max(1, int(args.probe_interval)),
        ),
    )

    def _shutdown() -> None:
        logger.info("正在停止桥接：请求机器人 STOP_POLICY 并关闭连接...")
        bridge.stop()
        try:
            robot_client.stop_policy()
        except Exception:  # noqa: BLE001
            logger.warning("STOP_POLICY 发送失败（可能连接已断）。")
        right_cam.close()
        if top_cam is not None:
            top_cam.close()
        robot_client.close()
        ws_client.close()

    try:
        bridge.run_forever()
    except KeyboardInterrupt:
        logger.info("收到 Ctrl+C，开始优雅停止。")
    finally:
        _shutdown()
