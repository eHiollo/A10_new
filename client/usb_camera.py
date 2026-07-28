from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# 多路 USB 相机同总线时，不宜长期同时 open；逐帧独占打开更稳（与 capture_cameras 一致）。
_USB_CAPTURE_LOCK = threading.Lock()


def _resize_with_pad(image: np.ndarray, target_h: int, target_w: int) -> np.ndarray:
    h, w = image.shape[:2]
    if h <= 0 or w <= 0:
        return np.zeros((target_h, target_w, 3), dtype=np.uint8)

    scale = min(target_w / w, target_h / h)
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    y0 = (target_h - new_h) // 2
    x0 = (target_w - new_w) // 2
    canvas[y0 : y0 + new_h, x0 : x0 + new_w] = resized
    return canvas


def _parse_source(source: str) -> int | str:
    return int(source) if source.isdigit() else source


def _configure_capture(cap: cv2.VideoCapture, width: int, height: int) -> None:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)


def _bgr_to_rgb(frame_bgr: np.ndarray, rotate_180: bool) -> np.ndarray:
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    if rotate_180:
        rgb = cv2.rotate(rgb, cv2.ROTATE_180)
    return rgb


@dataclass
class CameraConfig:
    source: str
    width: int = 640
    height: int = 480
    """若为 True，在 BGR→RGB 后对整帧做 180° 旋转（倒装相机）。"""
    rotate_180: bool = False
    """每帧独占打开/读取/释放，避免多相机同时占用 USB 导致读帧失败。"""
    exclusive_per_read: bool = False


class USBCamera:
    """Simple USB camera wrapper (supports /dev/videoX or numeric index)."""

    def __init__(self, cfg: CameraConfig) -> None:
        self._cfg = cfg
        self._cap: cv2.VideoCapture | None = None
        if not self._cfg.exclusive_per_read:
            self._open_persistent()

    def _open_persistent(self) -> None:
        source = _parse_source(self._cfg.source)
        self._cap = cv2.VideoCapture(source)
        if self._cap is not None:
            _configure_capture(self._cap, self._cfg.width, self._cfg.height)

        if self._cap is None or not self._cap.isOpened():
            raise RuntimeError(f"Failed to open USB camera source={self._cfg.source}")

        logger.info(
            "USB camera opened: source=%s size=%dx%d",
            self._cfg.source,
            int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )

    def _read_from_cap(self, cap: cv2.VideoCapture, warmup_frames: int = 2) -> np.ndarray:
        for _ in range(warmup_frames):
            cap.grab()
        ok, frame_bgr = cap.read()
        if not ok or frame_bgr is None:
            raise RuntimeError(f"Failed to read frame from USB camera source={self._cfg.source}")
        return _bgr_to_rgb(frame_bgr, self._cfg.rotate_180)

    def _read_exclusive(self) -> np.ndarray:
        source = _parse_source(self._cfg.source)
        with _USB_CAPTURE_LOCK:
            cap = cv2.VideoCapture(source)
            if not cap.isOpened():
                raise RuntimeError(f"Failed to open USB camera source={self._cfg.source}")
            try:
                _configure_capture(cap, self._cfg.width, self._cfg.height)
                return self._read_from_cap(cap, warmup_frames=2)
            finally:
                cap.release()

    def _read_persistent(self) -> np.ndarray:
        if self._cap is None or not self._cap.isOpened():
            self._open_persistent()
        try:
            return self._read_from_cap(self._cap, warmup_frames=0)
        except RuntimeError:
            logger.warning("USB camera read failed, reopening source=%s", self._cfg.source)
            self.close()
            self._open_persistent()
            return self._read_from_cap(self._cap, warmup_frames=2)

    def read_rgb(self) -> np.ndarray:
        if self._cfg.exclusive_per_read:
            return self._read_exclusive()
        return self._read_persistent()

    @staticmethod
    def preprocess_to_policy_chw(frame_rgb: np.ndarray, size: int = 224) -> np.ndarray:
        frame = np.asarray(frame_rgb)
        if frame.ndim != 3 or frame.shape[-1] < 3 or frame.size == 0:
            frame = np.zeros((size, size, 3), dtype=np.uint8)
        frame = frame[..., :3]
        if np.issubdtype(frame.dtype, np.floating):
            max_v = float(np.max(frame)) if frame.size > 0 else 0.0
            if max_v <= 1.01:
                frame = (np.clip(frame, 0.0, 1.0) * 255.0).round().astype(np.uint8)
            else:
                frame = np.clip(frame, 0.0, 255.0).round().astype(np.uint8)
        else:
            frame = frame.astype(np.uint8, copy=False)

        frame = _resize_with_pad(frame, size, size)
        chw = np.transpose(frame, (2, 0, 1))
        return np.ascontiguousarray(chw, dtype=np.uint8)

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None


class ThreadedCamera:
    """后台线程持续抓帧的相机封装，``read_rgb`` 立即返回最新帧。

    用于异步推理：避免每次观测都 open/read/close（exclusive 模式 100–300ms），
    让采图不再成为推理/执行重叠的瓶颈。后台线程持有持久句柄，把最新帧写入
    双缓冲（写第 2 个 buffer 时持锁，读侧取第 1 个），读侧无锁拿最新已写完的帧。
    """

    def __init__(self, cfg: CameraConfig) -> None:
        self._cfg = cfg
        self._stop = threading.Event()
        self._frame_lock = threading.Lock()
        self._latest: np.ndarray | None = None
        self._open_error: Exception | None = None
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self) -> None:
        source = _parse_source(self._cfg.source)
        cap: cv2.VideoCapture | None = None
        while not self._stop.is_set():
            try:
                if cap is None or not cap.isOpened():
                    if cap is not None:
                        cap.release()
                    # exclusive 模式下多相机同总线时这里仍可能失败；按需重试。
                    if self._cfg.exclusive_per_read:
                        with _USB_CAPTURE_LOCK:
                            cap = cv2.VideoCapture(source)
                    else:
                        cap = cv2.VideoCapture(source)
                    if not cap.isOpened():
                        raise RuntimeError(f"Failed to open USB camera source={self._cfg.source}")
                    _configure_capture(cap, self._cfg.width, self._cfg.height)
                    self._open_error = None
                ok, frame_bgr = cap.read()
                if not ok or frame_bgr is None:
                    # 丢帧/掉线：重置句柄下一轮重开。
                    cap.release()
                    cap = None
                    continue
                frame_rgb = _bgr_to_rgb(frame_bgr, self._cfg.rotate_180)
                with self._frame_lock:
                    self._latest = frame_rgb
            except Exception as exc:  # noqa: BLE001
                self._open_error = exc
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:  # noqa: BLE001
                        pass
                    cap = None
                if not self._stop.is_set():
                    time.sleep(0.2)
        if cap is not None:
            cap.release()

    def read_rgb(self) -> np.ndarray:
        # 等待第一帧（最多 ~2s），之后始终返回最新可用帧。
        t0 = time.time()
        while True:
            with self._frame_lock:
                frame = self._latest
                err = self._open_error
            if frame is not None:
                return frame
            if err is not None and time.time() - t0 > 2.0:
                raise RuntimeError(f"ThreadedCamera source={self._cfg.source} failed: {err}")
            if time.time() - t0 > 5.0:
                raise RuntimeError(f"ThreadedCamera source={self._cfg.source} timed out waiting first frame")
            time.sleep(0.02)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
