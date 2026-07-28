"""AsyncVLA 候选记录器：把每轮批量采样结果落盘为 jsonl，供 pilot 分析。

每行一条记录，包含候选全量数组——维度构成分析（校准夹爪权重）需要
原始候选，divergence 等标量只是派生量。

数据量估算：N=4, T=10, D=7，每行约 3-5KB；10Hz 下每分钟约 2-3MB，
pilot 规模（几十分钟）完全可接受。
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time

import numpy as np

logger = logging.getLogger(__name__)


class CandidateLogger:
    """线程安全的候选 jsonl 记录器（inferencer 线程单写者，仍加锁保底）。"""

    def __init__(self, log_dir: str, ndigits: int = 5) -> None:
        os.makedirs(log_dir, exist_ok=True)
        self._path = os.path.join(log_dir, f"candidates_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
        self._file = open(self._path, "a", encoding="utf-8")
        self._lock = threading.Lock()
        self._ndigits = ndigits
        logger.info("候选记录写入: %s", self._path)

    @property
    def path(self) -> str:
        return self._path

    def log(
        self,
        cycle: int,
        candidates: np.ndarray,
        best_index: int,
        divergence: float,
        per_candidate_score: np.ndarray,
        verifier: str = "medoid",
    ) -> None:
        """写入一条记录。

        ``per_candidate_score``: medoid 模式为各候选到其他候选的平均距离（越小越优），
        critic 模式为锦标赛胜场数（越大越优）；``verifier`` 字段标明口径。
        ``divergence`` 两种模式均为几何口径（跨实现一致, 驱动 adaptive N）。
        """
        record = {
            "cycle": int(cycle),
            "t": round(time.time(), 3),
            "sample_n": int(candidates.shape[0]),
            "verifier": verifier,
            "best_index": int(best_index),
            "divergence": round(float(divergence), 6),
            "per_cand_score": np.round(np.asarray(per_candidate_score, dtype=np.float64), 6).tolist(),
            "candidates": np.round(np.asarray(candidates, dtype=np.float64), self._ndigits).tolist(),
        }
        line = json.dumps(record, ensure_ascii=False)
        with self._lock:
            self._file.write(line + "\n")
            self._file.flush()

    def close(self) -> None:
        with self._lock:
            if not self._file.closed:
                self._file.close()
