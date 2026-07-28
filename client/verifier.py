"""AsyncVLA Level-1 verifier: geometric-consistency medoid selection.

假设：VLA 对同一观测多次采样时，正确的 action chunk 会形成聚簇，错误/低质量
候选更倾向于成为离群点。因此选取与其他候选平均距离最小的 medoid 作为最优。

零训练、零额外模型，仅依赖候选间的几何结构，是 AsyncVLA 三件套中的
Level-1（保底）verifier；Level-2（学习式 Relative Action Critic）后续在
本模块中以统一 ``select`` 接口扩展。
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class VerificationResult:
    """一次候选验证的结果。"""

    # 被选中的最优 action chunk，形状 (T, D)。
    best: np.ndarray
    # 最优候选在输入中的索引。
    best_index: int
    # 候选间分歧度：所有候选的两两距离均值。越大说明策略越不确定，
    # 后续可作为 adaptive sample_n 与主动求助（HIL）的触发信号。
    divergence: float
    # 每个候选到其他候选的平均距离，形状 (N,)。
    per_candidate_mean_dist: np.ndarray = field(repr=False)


class GeometricMedoidVerifier:
    """L1 几何一致性 medoid 验证器。

    距离度量：两个 chunk 间逐维加权的 L2 距离，对 horizon 取均值。
    权重用于抹平量纲差异——A10 前 6 维为关节 (rad，量级 ~1)，第 7 维
    夹爪为绝对位置 (mm，量级 ~100)，不加权时夹爪会主导距离。
    """

    def __init__(self, dim_weights: np.ndarray | None = None) -> None:
        self._w = None if dim_weights is None else np.asarray(dim_weights, dtype=np.float64).reshape(-1)

    def select(self, candidates: np.ndarray) -> VerificationResult:
        c = np.asarray(candidates, dtype=np.float64)
        if c.ndim != 3:
            raise ValueError(f"Expected candidates with shape (N, T, D), got {c.shape}")
        n = c.shape[0]
        if n == 1:
            return VerificationResult(
                best=c[0].astype(np.float32),
                best_index=0,
                divergence=0.0,
                per_candidate_mean_dist=np.zeros(1, dtype=np.float64),
            )

        w = self._w
        if w is None:
            w = np.ones(c.shape[-1], dtype=np.float64)
        elif w.shape[0] != c.shape[-1]:
            raise ValueError(f"dim_weights length {w.shape[0]} != action dim {c.shape[-1]}")

        # (N, N, T, D) 两两差 → 加权 L2 沿 D 聚合 → 对 T 取均值 → (N, N)
        diff = (c[:, None, :, :] - c[None, :, :, :]) * w
        dist = np.linalg.norm(diff, axis=-1).mean(axis=-1)

        # 对角线为 0，除以 n-1 得到每个候选到其他候选的平均距离。
        per_cand = dist.sum(axis=1) / (n - 1)
        best_idx = int(np.argmin(per_cand))

        # 分歧度 = 两两距离均值（排除对角零值，取严格上三角）。
        # 含对角会让零值占比随 N 变化（N=2 时 50%、N=8 时 12.5%），
        # 导致不同采样数下的 divergence 口径不一致；排除后跨 N 可比，
        # 才能作为 adaptive sample_n / HIL 求助的统一触发信号。
        divergence = float(dist[np.triu_indices(n, k=1)].mean())

        return VerificationResult(
            best=c[best_idx].astype(np.float32),
            best_index=best_idx,
            divergence=divergence,
            per_candidate_mean_dist=per_cand,
        )
