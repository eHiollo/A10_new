#!/usr/bin/env python3
"""Entry point for the A10 <-> OpenPI bridge.

支持两种启动方式（均通过把 client/ 注入 sys.path 后用平铺导入实现）：

* ``cd client && python run_bridge.py ...``   (传统)
* ``python client/run_bridge.py ...``         (从仓库根目录)
* ``python -m client.run_bridge ...``         (包方式, 从仓库根目录)
"""
from __future__ import annotations

import logging
import os
import sys

# 无论以何种方式启动，都把本文件所在目录加入 sys.path，使同级模块
# (bridge / usb_camera / ...) 可按平铺名导入。这样 ``python -m`` 与
# 直接运行脚本两种方式都能正常工作。
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from bridge import build_arg_parser, run_from_args  # noqa: E402


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = build_arg_parser()
    args = parser.parse_args()
    run_from_args(args)


if __name__ == "__main__":
    main()
