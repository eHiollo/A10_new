"""AsyncVLA Level-2 verifier: 学习式 Relative Action Critic（锦标赛选优）。

加载 lerobot 侧 ``train_relative_critic`` 训练的 checkpoint，对 N 个候选
action chunk 两两比较累计胜场，胜场最多者胜出。

本文件自包含网络定义（``_RACNet``），与
``lerobot/policies/relative_critic/modeling_relative_critic.py`` 的
``RelativeActionCritic`` 结构保持一致（state_dict 键名兼容）——修改
任一侧时必须同步另一侧。bridge 不依赖 lerobot 包，仅需 torch（lazy import，
机器人 PC 无 torch 时给出清晰报错）。

分歧度信号：critic 自身无天然 divergence，bridge 端仍用几何
``compute_divergence`` 驱动 adaptive N 与日志；本类 ``select`` 返回的
win_counts 存入日志便于离线分析 critic 置信度。
"""
from __future__ import annotations

import logging

import numpy as np

logger = logging.getLogger(__name__)


def _build_net(cfg: dict):
    """按 config dict 构建 RAC 网络（结构须与 lerobot 侧一致）。"""
    import torch.nn as nn

    state_dim = int(cfg.get("state_dim", 7))
    action_dim = int(cfg.get("action_dim", 7))
    horizon = int(cfg.get("action_horizon", 10))
    state_hidden = int(cfg.get("state_hidden", 128))
    action_hidden = int(cfg.get("action_hidden", 256))
    pair_hidden = int(cfg.get("pair_hidden", 256))

    state_enc = nn.Sequential(
        nn.Linear(state_dim, state_hidden), nn.ReLU(),
        nn.Linear(state_hidden, state_hidden), nn.ReLU(),
    )
    action_enc = nn.Sequential(
        nn.Linear(horizon * action_dim, action_hidden), nn.ReLU(),
        nn.Linear(action_hidden, action_hidden), nn.ReLU(),
    )
    pair_head = nn.Sequential(
        nn.Linear(state_hidden + 3 * action_hidden, pair_hidden), nn.ReLU(),
        nn.Dropout(0.0),  # 推理态, dropout 值不影响 eval 行为, 仅占位保证键名兼容
        nn.Linear(pair_hidden, pair_hidden), nn.ReLU(),
        nn.Linear(pair_hidden, 1),
    )

    class _RACNet(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.state_enc = state_enc
            self.action_enc = action_enc
            self.pair_head = pair_head

        def forward(self, state, action_a, action_b):
            s = self.state_enc(state)
            fa = self.action_enc(action_a.flatten(start_dim=1))
            fb = self.action_enc(action_b.flatten(start_dim=1))
            import torch
            x = torch.cat([s, fa, fb, fa - fb], dim=-1)
            return self.pair_head(x).squeeze(-1)

    return _RACNet()


class RelativeCriticVerifier:
    """L2 critic 验证器：从 checkpoint 加载, 锦标赛选优。

    与 ``GeometricMedoidVerifier`` 并存于 bridge：``--verifier medoid|critic``
    切换。接口差异——critic 需要当前 state 作为条件。
    """

    def __init__(self, checkpoint_path: str, device: str = "cpu") -> None:
        try:
            import torch
        except ImportError as e:
            raise RuntimeError(
                "RelativeCriticVerifier 需要 torch；请在机器人 PC 安装 torch (CPU 版即可), "
                "或改用 --verifier medoid"
            ) from e
        self._torch = torch
        self._device = torch.device(device)
        ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
        cfg = ckpt.get("config", {})
        self._net = _build_net(cfg)
        self._net.load_state_dict(ckpt["model"])
        self._net.to(self._device).eval()
        logger.info(
            "L2 critic 已加载: %s (val_acc=%.3f, device=%s)",
            checkpoint_path, float(ckpt.get("val_acc", float("nan"))), self._device,
        )

    def select(self, state: np.ndarray, candidates: np.ndarray) -> tuple[int, np.ndarray]:
        """锦标赛选优: 返回 (best_index, win_counts)。

        state: (7,) 当前观测（绝对关节）；candidates: (N, T, 7) 绝对关节 chunk。
        """
        torch = self._torch
        c = np.asarray(candidates, dtype=np.float32)
        n = c.shape[0]
        if n == 1:
            return 0, np.zeros(1, dtype=np.int64)
        with torch.no_grad():
            ct = torch.as_tensor(c, device=self._device)
            st = torch.as_tensor(np.asarray(state, dtype=np.float32), device=self._device)
            wins = torch.zeros(n, dtype=torch.long, device=self._device)
            for i in range(n):
                for j in range(i + 1, n):
                    logit = self._net(st.unsqueeze(0), ct[i].unsqueeze(0), ct[j].unsqueeze(0))
                    if bool(logit.item() > 0):
                        wins[i] += 1
                    else:
                        wins[j] += 1
        return int(wins.argmax().item()), wins.cpu().numpy()
