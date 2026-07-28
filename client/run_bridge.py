#!/usr/bin/env python3
from __future__ import annotations

import logging

from bridge import build_arg_parser, run_from_args


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
