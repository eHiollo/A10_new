"""AsyncVLA 候选记录分析：divergence 分布 / 维度构成 / verifier 选择统计。

用法:
    python3 analyze_candidates.py <candidates_xxx.jsonl> [--gripper-weight 0.03]

核心输出:
1. divergence 分布(均值/分位/最大) —— 标定 adaptive N 阈值的依据
2. 各维度对两两距离的贡献占比(不加权口径) —— 校准夹爪权重的直接证据:
   若夹爪维贡献占比 >> 1/7, 说明其量级确实主导距离, 当前降权合理;
   报告同时给出"等效同量级"所需的权重建议值
3. best_index 分布 —— medoid 选择是否稳定偏向某些候选(异常则为 bug 信号)
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np


def _load(path: str) -> list[dict]:
    records = []
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                print(f"警告: 第 {lineno} 行解析失败, 跳过 ({exc})", file=sys.stderr)
    return records


def _dim_contribution(candidates: np.ndarray) -> np.ndarray:
    """各维度对两两距离的贡献(不加权): mean_{i<j} |c_i - c_j| 沿 T 平均后按维分解。

    返回 (D,) 贡献占比, 和为 1。
    """
    c = np.asarray(candidates, dtype=np.float64)
    n = c.shape[0]
    if n < 2:
        return np.zeros(c.shape[-1])
    # (N,N,T,D) 绝对差沿 T 平均 → (N,N,D) → 上三角均值 → (D,)
    abs_diff = np.abs(c[:, None, :, :] - c[None, :, :, :]).mean(axis=2)
    iu = np.triu_indices(n, k=1)
    per_dim = abs_diff[iu].mean(axis=0)
    total = per_dim.sum()
    return per_dim / total if total > 0 else per_dim


def main() -> None:
    parser = argparse.ArgumentParser(description="分析 AsyncVLA 候选记录 jsonl")
    parser.add_argument("log", type=str, help="candidates_*.jsonl 路径")
    parser.add_argument("--gripper-weight", type=float, default=0.03, help="当前使用的夹爪权重(用于对照)")
    args = parser.parse_args()

    records = _load(args.log)
    if not records:
        print("无有效记录。")
        return

    divs = np.array([r["divergence"] for r in records], dtype=np.float64)
    best_idx = np.array([r["best_index"] for r in records])
    n_candidates = records[0]["sample_n"]
    cands0 = np.asarray(records[0]["candidates"])
    n_records = len(records)
    t0, t1 = records[0]["t"], records[-1]["t"]

    print("=" * 64)
    print(f"记录: {args.log}")
    print(f"轮数: {n_records}  采样数 N: {n_candidates}  chunk: {cands0.shape[1]}x{cands0.shape[2]}")
    print(f"时长: {(t1 - t0):.1f}s  频率: {n_records / max(t1 - t0, 1e-6):.2f} 轮/s")
    print("=" * 64)

    # ---- 1. divergence 分布 ----
    print("\n[1] divergence 分布(加权口径, 与运行时 verifier 一致)")
    qs = np.percentile(divs, [10, 50, 90, 99])
    print(f"  mean={divs.mean():.4f}  std={divs.std():.4f}")
    print(f"  p10={qs[0]:.4f}  p50={qs[1]:.4f}  p90={qs[2]:.4f}  p99={qs[3]:.4f}  max={divs.max():.4f}")
    print("  → adaptive N 阈值参考: low 可取 p50 附近, high 可取 p90 附近")

    # ---- 2. 维度贡献构成 ----
    print("\n[2] 各维度两两距离贡献占比(不加权原始口径, 全轮平均)")
    dim_names = [f"joint_{i + 1}" for i in range(cands0.shape[2] - 1)] + ["gripper"]
    contrib = np.mean([_dim_contribution(r["candidates"]) for r in records], axis=0)
    uniform = 1.0 / len(contrib)
    for name, c in zip(dim_names, contrib):
        bar = "#" * int(c * 100)
        flag = "  << 显著超出均匀占比" if c > 3 * uniform else ""
        print(f"  {name:>9}: {c * 100:5.1f}%  {bar}{flag}")
    grip_share = contrib[-1]
    if grip_share > 3 * uniform:
        print(f"  → 夹爪主导距离({grip_share * 100:.1f}% vs 均匀 {uniform * 100:.1f}%), 当前降权 {args.gripper_weight} 方向正确")
    elif grip_share < uniform / 3:
        print(f"  → 夹爪贡献偏低({grip_share * 100:.1f}%), 可考虑上调权重或检查夹爪是否几乎不动")

    # ---- 3. best_index 分布 ----
    print("\n[3] verifier 选择分布")
    counts = np.bincount(best_idx, minlength=n_candidates)
    for i, cnt in enumerate(counts):
        print(f"  候选 {i}: {cnt:>5} 次 ({cnt / n_records * 100:5.1f}%)")
    if counts.max() / n_records > 0.9:
        print("  ⚠ 选择极度偏向单一候选——若候选由相同 noise 产生, 需检查服务端采样是否退化")

    print()


if __name__ == "__main__":
    main()
