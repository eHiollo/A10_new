"""GeometricMedoidVerifier 回归测试。

运行: cd client && python -m pytest tests/test_verifier.py -v
(无 pytest 时: python tests/test_verifier.py)
"""
from __future__ import annotations

import os
import sys

import numpy as np

# 兼容从 client/ 或 client/tests/ 运行: 把 client/ 注入 sys.path。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from verifier import GeometricMedoidVerifier  # noqa: E402


def _make_cluster_outlier(seed: int = 0, horizon: int = 10, dim: int = 7):
    """3 个紧密聚簇候选 + 1 个离群候选。"""
    rng = np.random.default_rng(seed)
    base = rng.normal(size=(horizon, dim)).astype(np.float32)
    cluster = np.stack([base + rng.normal(scale=0.01, size=(horizon, dim)) for _ in range(3)])
    outlier = base + rng.normal(scale=1.0, size=(horizon, dim))
    return np.concatenate([cluster, outlier[None]], axis=0), base


def test_medoid_selects_cluster_member():
    cands, _ = _make_cluster_outlier()
    r = GeometricMedoidVerifier(dim_weights=np.ones(7)).select(cands)
    assert r.best_index in (0, 1, 2)
    assert r.best.shape == (10, 7)
    assert r.best.dtype == np.float32
    assert r.per_candidate_mean_dist.shape == (4,)
    # 离群候选的平均距离应显著大于聚簇成员(理论上限 3 倍: 聚簇内距离→0 时)
    assert r.per_candidate_mean_dist[3] > 2 * r.per_candidate_mean_dist[r.best_index]


def test_gripper_weight_dominance():
    """夹爪维(mm 量级)不加权时会主导距离; 降权后应被正确忽略。"""
    cands, base = _make_cluster_outlier()
    # 把离群替换为仅夹爪维 +50mm 的候选(关节完全正常)
    cands = np.concatenate([cands[:3], (base + np.array([0, 0, 0, 0, 0, 0, 50.0])[None, :])[None]], axis=0)
    r_weighted = GeometricMedoidVerifier(dim_weights=np.array([1, 1, 1, 1, 1, 1, 0.03])).select(cands)
    assert r_weighted.best_index in (0, 1, 2)


def test_single_candidate_passthrough():
    cands, _ = _make_cluster_outlier()
    r = GeometricMedoidVerifier(dim_weights=np.ones(7)).select(cands[:1])
    assert r.best_index == 0
    assert r.divergence == 0.0
    np.testing.assert_array_equal(r.per_candidate_mean_dist, np.zeros(1))


def test_divergence_excludes_diagonal():
    """divergence 必须等于严格上三角均值(排除对角零值), 保证跨 N 口径一致。"""
    cands, _ = _make_cluster_outlier()
    v = GeometricMedoidVerifier(dim_weights=np.ones(7))
    r = v.select(cands)
    d = np.linalg.norm((cands[:, None] - cands[None, :]).astype(np.float64), axis=-1).mean(axis=-1)
    np.testing.assert_allclose(r.divergence, d[np.triu_indices(4, k=1)].mean(), rtol=1e-9)

    r2 = v.select(cands[:2])
    d2 = d[:2, :2]
    np.testing.assert_allclose(r2.divergence, d2[np.triu_indices(2, k=1)].mean(), rtol=1e-9)


def test_identical_candidates_zero_divergence():
    _, base = _make_cluster_outlier()
    r = GeometricMedoidVerifier(dim_weights=np.ones(7)).select(np.stack([base] * 4))
    assert r.divergence < 1e-6


def test_dim_weights_length_mismatch():
    cands, _ = _make_cluster_outlier()
    v = GeometricMedoidVerifier(dim_weights=np.ones(5))
    try:
        v.select(cands)
        raise AssertionError("should raise ValueError")
    except ValueError:
        pass


def test_invalid_ndim():
    v = GeometricMedoidVerifier(dim_weights=np.ones(7))
    try:
        v.select(np.zeros((10, 7), dtype=np.float32))
        raise AssertionError("should raise ValueError")
    except ValueError:
        pass


if __name__ == "__main__":
    for name, fn in sorted(list(globals().items())):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS {name}")
    print("ALL TESTS PASSED")
