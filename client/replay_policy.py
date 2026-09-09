"""Replay dataset joint trajectories as fake policy inference.

Used to verify A10 async bridging without GPU, cameras, or gripper.
Each ``infer()`` sleeps ``clip(N(μ, σ), lo, hi)`` then returns the next
``horizon`` rows of ``observation.state`` as ``{"actions": (T, 7)}``.
"""
from __future__ import annotations

import glob
import logging
import os
import time
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

_DEFAULT_DATASET = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "dataset", "dataset_5_9")
)


class EndOfReplay(Exception):
    """Episode exhausted; bridge should stop cleanly."""


class ReplayPolicy:
    def __init__(
        self,
        dataset_root: str | None = None,
        episode: int = 15,
        start_frame: int = 0,
        horizon: int = 10,
        infer_mean_ms: float = 68.5,
        infer_std_ms: float = 0.7,
        infer_clip_ms: tuple[float, float] = (20.0, 150.0),
        seed: int = 0,
        stride: int | None = None,
    ) -> None:
        self._horizon = int(horizon)
        self._mean = float(infer_mean_ms)
        self._std = float(infer_std_ms)
        self._clip = (float(infer_clip_ms[0]), float(infer_clip_ms[1]))
        self._rng = np.random.default_rng(seed)
        self.last_infer_ms = 0.0
        root = dataset_root or _DEFAULT_DATASET
        self._states = _load_episode_states(root, int(episode), int(start_frame))
        self._stride = max(1, int(stride) if stride is not None else self._horizon)
        self._cursor = 0
        logger.info(
            "ReplayPolicy: episode=%d start_frame=%d n=%d horizon=%d stride=%d delay=clip(N(%.1f, %.1f), %.0f, %.0f) ms",
            episode,
            start_frame,
            len(self._states),
            self._horizon,
            self._stride,
            self._mean,
            self._std,
            self._clip[0],
            self._clip[1],
        )

    def infer(self, observation: dict[str, Any], sample_n: int = 1) -> dict[str, Any]:
        del observation, sample_n
        if self._cursor >= len(self._states):
            raise EndOfReplay("replay episode exhausted")
        delay_ms = float(np.clip(self._rng.normal(self._mean, self._std), self._clip[0], self._clip[1]))
        time.sleep(delay_ms / 1000.0)
        self.last_infer_ms = delay_ms
        end = min(self._cursor + self._horizon, len(self._states))
        chunk = np.ascontiguousarray(self._states[self._cursor : end], dtype=np.float32)
        self._cursor = min(len(self._states), self._cursor + self._stride)
        return {"actions": chunk, "policy_timing": {"infer_ms": delay_ms}}

    def close(self) -> None:
        return


def _load_episode_states(dataset_root: str, episode: int, start_frame: int) -> np.ndarray:
    pattern = os.path.join(dataset_root, "data", "chunk-*", f"episode_{episode:06d}.parquet")
    matches = sorted(glob.glob(pattern))
    if not matches:
        raise FileNotFoundError(f"episode {episode} parquet not found under {dataset_root} ({pattern})")
    path = matches[0]
    try:
        import pyarrow.parquet as pq
    except ImportError as exc:
        raise ImportError("ReplayPolicy 需要 pyarrow 读 LeRobot parquet：pip install pyarrow") from exc
    table = pq.read_table(path, columns=["observation.state"])
    col = table.column("observation.state")
    rows = []
    for i in range(table.num_rows):
        v = np.asarray(col[i].as_py(), dtype=np.float32).reshape(-1)
        if v.shape[0] < 7:
            v = np.concatenate([v, np.zeros((7 - v.shape[0],), dtype=np.float32)])
        rows.append(v[:7])
    arr = np.stack(rows, axis=0)
    if start_frame < 0 or start_frame >= len(arr):
        raise ValueError(f"start_frame={start_frame} out of range for episode {episode} n={len(arr)}")
    logger.info("Loaded replay states from %s shape=%s", path, arr[start_frame:].shape)
    return arr[start_frame:]
