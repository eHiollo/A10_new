"""Persistent diagnostics for the real robot VLA bridge."""

from __future__ import annotations

import json
import logging
import os
import tempfile
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any

_timing_logger = logging.getLogger("vla_timing")


def configure_logging(log_dir: str | Path, *, enabled: bool = True) -> Path | None:
    """Start a fresh log session, or keep only console output when disabled."""
    for old_handler in _timing_logger.handlers[:]:
        _timing_logger.removeHandler(old_handler)
        old_handler.close()
    _timing_logger.setLevel(logging.INFO)
    _timing_logger.propagate = False

    formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logging.basicConfig(level=logging.INFO, handlers=[console_handler], force=True)

    if not enabled:
        logging.getLogger(__name__).info("VLA file logging disabled.")
        return None

    directory = Path(log_dir).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    # Atomic creation prevents collisions even for simultaneous process starts.
    session_dir = Path(tempfile.mkdtemp(prefix=f"run_{timestamp}_pid{os.getpid()}_", dir=directory))

    text_handler = RotatingFileHandler(
        session_dir / "vla_bridge.log", maxBytes=20 * 1024 * 1024, backupCount=5, encoding="utf-8"
    )
    text_handler.setFormatter(formatter)
    logging.getLogger().addHandler(text_handler)

    timing_handler = RotatingFileHandler(
        session_dir / "vla_timing.jsonl", maxBytes=20 * 1024 * 1024, backupCount=5, encoding="utf-8"
    )
    timing_handler.setFormatter(logging.Formatter("%(message)s"))
    _timing_logger.addHandler(timing_handler)
    logging.getLogger(__name__).info("VLA session logs: %s", session_dir)
    return session_dir


def record_timing_event(event: str, **fields: Any) -> None:
    """Write one UTC timestamped JSON object, without image or action payloads."""
    if not _timing_logger.isEnabledFor(logging.INFO) or not _timing_logger.handlers:
        return
    item = {"ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds"), "event": event, **fields}
    _timing_logger.info(json.dumps(item, ensure_ascii=False, separators=(",", ":")))


class PolicyLogSession:
    """Open and close file logging according to robot policy status."""

    def __init__(self, log_dir: str | Path) -> None:
        self.log_dir = log_dir
        self.session_id: str | None = None
        self.session_dir: Path | None = None
        self._warned_missing_status = False

    def observe(self, status: dict) -> None:
        """Apply state changes seen while waiting for robot execution."""
        self.sync(status)

    def sync(self, status: dict) -> None:
        """Close old files and start the current policy session when observed."""
        required = ("policy_running", "log_enabled", "log_session")
        if not all(key in status for key in required):
            self.close(reason="unsupported_robot_status")
            if not self._warned_missing_status:
                logging.getLogger(__name__).warning(
                    "Robot status lacks policy logging fields; rebuild/restart the robot "
                    "with the updated kaanh.xml. File logging remains disabled."
                )
                self._warned_missing_status = True
            return

        session = status["log_session"]
        enabled = (status["policy_running"] is True and status["log_enabled"] is True
                   and isinstance(session, str) and session not in ("", "0"))
        if self.session_id is not None and (not enabled or session != self.session_id):
            self.close(reason="policy_stopped_or_replaced")
        if enabled and self.session_id is None:
            self.session_dir = configure_logging(self.log_dir, enabled=True)
            self.session_id = session
            record_timing_event("policy_session_start", policy_session=session)
            logging.getLogger(__name__).info("Following robot policy log session %s", session)

    def close(self, *, reason: str = "closed") -> None:
        if self.session_id is None:
            return
        record_timing_event("policy_session_end", policy_session=self.session_id, reason=reason)
        configure_logging(self.log_dir, enabled=False)
        self.session_id = None
        self.session_dir = None
