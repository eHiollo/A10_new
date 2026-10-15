from __future__ import annotations

from collections import deque

import numpy as np


class TemporalEnsemble:
    def __init__(self, decay: float = 0.01, include_gripper: bool = True):
        if not np.isfinite(decay) or decay < 0.0:
            raise ValueError("ensemble decay must be finite and nonnegative")
        self.decay = decay
        self.include_gripper = include_gripper
        self._history = deque(maxlen=32)

    def clear(self):
        self._history.clear()

    def fuse(self, actions, observed_at: float, step_s: float):
        raw = np.asarray(actions, dtype=float)
        if raw.ndim != 2 or raw.shape[1] != 7 or not len(raw) or not np.isfinite(raw).all():
            raise ValueError("ensemble actions must be finite (T, 7)")
        if not np.isfinite(observed_at) or not np.isfinite(step_s) or step_s <= 0:
            raise ValueError("invalid ensemble timestamps")
        if self._history and (observed_at <= self._history[-1][0] or step_s != self._history[-1][1]):
            self.clear()
        self._history = deque((entry for entry in self._history
                               if entry[0] + (len(entry[2]) - 1) * entry[1] >= observed_at), maxlen=32)
        self._history.append((observed_at, step_s, raw.copy()))
        fused = raw.copy()
        counts = []
        for index in range(len(raw)):
            target_time = observed_at + index * step_s
            candidates = []
            for origin, period, chunk in self._history:
                coordinate = (target_time - origin) / period
                if coordinate < -1e-7 or coordinate > len(chunk) - 1 + 1e-7:
                    continue
                coordinate = np.clip(coordinate, 0, len(chunk) - 1)
                lower = int(np.floor(coordinate))
                upper = min(lower + 1, len(chunk) - 1)
                fraction = coordinate - lower
                candidates.append(chunk[lower] * (1.0 - fraction) + chunk[upper] * fraction)
            weights = np.exp(-self.decay * np.arange(len(candidates)))
            weights /= weights.sum()
            fused[index] = np.sum(np.asarray(candidates) * weights[:, None], axis=0)
            counts.append(len(candidates))
        if not self.include_gripper:
            fused[:, 6] = raw[:, 6]
        return fused, {"contributors_per_step": counts, "history_chunks": len(self._history),
                       "decay": self.decay, "weight_order": "oldest_first",
                       "include_gripper": self.include_gripper,
                       "max_delta_per_axis": np.max(np.abs(fused - raw), axis=0).tolist()}
