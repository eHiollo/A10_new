"""Sample follower joints + policy remaining during execution and flag freezes.

Used to debug slight hitch: distinguish expected sync-infer pauses (queue empty
~68ms) from unexpected freezes while the RT still has remaining keyframes.
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

# Below this peak joint speed the arm is treated as stopped.
_FREEZE_SPEED_RAD_S = 0.025
# Ignore blips shorter than the TCP state thread period (~33ms).
_MIN_FREEZE_MS = 40.0


class StutterMonitor:
    def __init__(
        self,
        robot,
        hz: float = 20.0,
        log_path: str | None = None,
        should_stop=None,
    ) -> None:
        self._robot = robot
        self._dt = 1.0 / max(1.0, float(hz))
        self._log_path = log_path
        self._should_stop = should_stop
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._samples: list[dict[str, Any]] = []
        self._events: list[dict[str, Any]] = []
        self._freeze: dict[str, Any] | None = None

    def start(self) -> None:
        if self._log_path:
            os.makedirs(os.path.dirname(os.path.abspath(self._log_path)) or ".", exist_ok=True)
            logger.info("卡顿监测 %.0fHz，轨迹写入 %s", 1.0 / self._dt, self._log_path)
        else:
            logger.info("卡顿监测 %.0fHz（仅日志，不写文件）。", 1.0 / self._dt)
        self._thread = threading.Thread(target=self._run, name="stutter-monitor", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._close_freeze(time.time())
        self._write_log()
        self._log_summary()

    def _run(self) -> None:
        prev_q: np.ndarray | None = None
        prev_t: float | None = None
        while not self._stop.is_set():
            if self._should_stop is not None and self._should_stop():
                break
            t0 = time.time()
            try:
                snap = self._robot.snapshot_motion()
            except Exception:  # noqa: BLE001
                time.sleep(self._dt)
                continue
            q = np.asarray(snap.get("q") or [], dtype=np.float64)
            if q.size < 6:
                time.sleep(self._dt)
                continue
            q6 = q[:6]
            speed = 0.0
            if prev_q is not None and prev_t is not None:
                dt = max(1e-4, float(snap["t"]) - prev_t)
                speed = float(np.max(np.abs(q6 - prev_q) / dt))
            prev_q = q6
            prev_t = float(snap["t"])
            rem = int(snap.get("remaining") or 0)
            rec = {
                "t": float(snap["t"]),
                "remaining": rem,
                "idle": bool(snap.get("idle")),
                "batch_seq": int(snap.get("batch_seq") or 0),
                "speed": speed,
                "q": q6.tolist(),
            }
            self._samples.append(rec)
            self._update_freeze(rec)
            elapsed = time.time() - t0
            remain = self._dt - elapsed
            if remain > 0:
                time.sleep(remain)

    def _update_freeze(self, rec: dict[str, Any]) -> None:
        frozen = rec["speed"] < _FREEZE_SPEED_RAD_S
        kind = "queue_empty" if rec["remaining"] <= 0 else "exec_freeze"
        if frozen:
            if self._freeze is None:
                self._freeze = {
                    "kind": kind,
                    "t0": rec["t"],
                    "t1": rec["t"],
                    "remaining0": rec["remaining"],
                    "speed_min": rec["speed"],
                    "batch_seq": rec["batch_seq"],
                }
            else:
                self._freeze["t1"] = rec["t"]
                self._freeze["speed_min"] = min(self._freeze["speed_min"], rec["speed"])
                if kind != self._freeze["kind"]:
                    self._close_freeze(rec["t"])
                    self._freeze = {
                        "kind": kind,
                        "t0": rec["t"],
                        "t1": rec["t"],
                        "remaining0": rec["remaining"],
                        "speed_min": rec["speed"],
                        "batch_seq": rec["batch_seq"],
                    }
            return
        self._close_freeze(rec["t"])

    def _close_freeze(self, t_now: float) -> None:
        ev = self._freeze
        self._freeze = None
        if ev is None:
            return
        dur_ms = (ev["t1"] - ev["t0"]) * 1000.0
        if dur_ms < _MIN_FREEZE_MS:
            return
        ev["dur_ms"] = dur_ms
        self._events.append(ev)
        logger.info(
            "卡顿 %s %.0fms rem=%s batch_seq=%s speed_min=%.4f rad/s",
            ev["kind"],
            dur_ms,
            ev["remaining0"],
            ev["batch_seq"],
            ev["speed_min"],
        )

    def _write_log(self) -> None:
        if not self._log_path:
            return
        payload = {
            "freeze_speed_rad_s": _FREEZE_SPEED_RAD_S,
            "min_freeze_ms": _MIN_FREEZE_MS,
            "events": self._events,
            "samples": self._samples,
        }
        with open(self._log_path, "w", encoding="utf-8") as f:
            json.dump(payload, f)
        logger.info("卡顿监测已写入 %s（%d 采样 / %d 事件）", self._log_path, len(self._samples), len(self._events))

    def _log_summary(self) -> None:
        if not self._samples:
            logger.warning("卡顿监测没有采到关节状态（TCP 可能一直超时）。")
            return
        t0 = self._samples[0]["t"]
        t1 = self._samples[-1]["t"]
        by_kind: dict[str, list[float]] = {"queue_empty": [], "exec_freeze": []}
        for ev in self._events:
            by_kind.setdefault(ev["kind"], []).append(ev["dur_ms"])
        empty = by_kind["queue_empty"]
        hitch = by_kind["exec_freeze"]

        def _stats(xs: list[float]) -> str:
            if not xs:
                return "无"
            arr = np.asarray(xs, dtype=np.float64)
            return f"n={len(xs)} median={float(np.median(arr)):.0f}ms max={float(np.max(arr)):.0f}ms"

        logger.info(
            "卡顿监测汇总：时长=%.1fs 采样=%d | 段间空队列(同步推理预期~68ms): %s | 段内冻结(异常卡顿): %s",
            t1 - t0,
            len(self._samples),
            _stats(empty),
            _stats(hitch),
        )
        if hitch:
            logger.info(
                "段内冻结是轻微卡顿的主要嫌疑：remaining>0 但关节几乎不动（插值段切换/第一拍 alpha=0/TCP 卡住）。"
            )
        elif empty:
            logger.info(
                "没有段内冻结；能感到的顿挫更像段与段之间 remaining=0 时的同步推理等待。"
            )
