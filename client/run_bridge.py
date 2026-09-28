#!/usr/bin/env python3
from __future__ import annotations

from bridge import build_arg_parser, run_from_args
from runtime_logging import configure_logging


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    configure_logging(args.log_dir, enabled=False)
    run_from_args(args)


if __name__ == "__main__":
    main()
