#!/usr/bin/env python3
from __future__ import annotations

import logging

from bridge import build_arg_parser, run_from_args
from runtime_logging import RuntimeLog


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = build_arg_parser()
    args = parser.parse_args()
    runtime_log = RuntimeLog(args.log_dir, vars(args))
    try:
        run_from_args(args, runtime_log=runtime_log)
    except BaseException as exc:
        runtime_log.write("session_error", error=repr(exc))
        raise
    finally:
        runtime_log.close()


if __name__ == "__main__":
    main()
