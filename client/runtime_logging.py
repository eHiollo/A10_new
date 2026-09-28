"""Persistent diagnostics for the real robot VLA bridge."""

from __future__ import annotations

import json
import logging
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any

_timing_logger = logging.getLogger("vla_timing")


def configure_logging(log_dir: str | Path) -> Path:
    """Write normal logs and machine-readable cycle timings to rotating files."""
    directory = Path(log_dir).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)

    text_handler = RotatingFileHandler(
        directory / "vla_bridge.log", maxBytes=20 * 1024 * 1024, backupCount=5, encoding="utf-8"
    )
    text_handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s"))
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s"))
    logging.basicConfig(level=logging.INFO, handlers=[console_handler, text_handler], force=True)

    for old_handler in _timing_logger.handlers[:]:
        _timing_logger.removeHandler(old_handler)
        old_handler.close()
    timing_handler = RotatingFileHandler(
        directory / "vla_timing.jsonl", maxBytes=20 * 1024 * 1024, backupCount=5, encoding="utf-8"
    )
    timing_handler.setFormatter(logging.Formatter("%(message)s"))
    _timing_logger.addHandler(timing_handler)
    _timing_logger.setLevel(logging.INFO)
    _timing_logger.propagate = False
    logging.getLogger(__name__).info("VLA logs: %s", directory)
    return directory


def record_timing_event(event: str, **fields: Any) -> None:
    """Write one UTC timestamped JSON object, without image or action payloads."""
    if not _timing_logger.isEnabledFor(logging.INFO) or not _timing_logger.handlers:
        return
    item = {"ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds"), "event": event, **fields}
    _timing_logger.info(json.dumps(item, ensure_ascii=False, separators=(",", ":")))
