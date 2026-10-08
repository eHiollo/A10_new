from __future__ import annotations

import datetime
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time

import cv2
import numpy as np


def _json_value(value):
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, np.ndarray):
        return _json_value(value.tolist())
    if isinstance(value, np.generic):
        return _json_value(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


class RuntimeLog:
    def __init__(self, root: str, config: dict) -> None:
        root_path = Path(root).expanduser().resolve()
        root_path.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.path = Path(tempfile.mkdtemp(prefix=f"run_{stamp}_pid{os.getpid()}_", dir=root_path))
        self._lock = threading.Lock()
        self._file = (self.path / "events.jsonl").open("w", buffering=1, encoding="utf-8")
        self._saved_images = False
        self._handler = logging.FileHandler(self.path / "bridge.log", encoding="utf-8")
        self._handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s"))
        logging.getLogger().addHandler(self._handler)
        safe_config = {key: value for key, value in config.items() if "key" not in key.lower() and "password" not in key.lower()}
        repo = Path(__file__).resolve().parent.parent
        try:
            revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
            status = subprocess.check_output(["git", "status", "--short"], cwd=repo, text=True)
        except (OSError, subprocess.CalledProcessError):
            revision, status = "unknown", "unknown"
        sources = [*Path(__file__).parent.glob("*.py"), repo / "a10_policy_tcp_plan.cpp", repo / "a10_tcp_server.cpp", repo / "a10_policy_batch.hpp", repo / "a10_policy_smoother.hpp"]
        hashes = {str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources if path.exists()}
        self.write("session_start", config=safe_config, revision=revision, git_status=status, source_sha256=hashes)
        logging.getLogger(__name__).info("本次实机日志：%s", self.path)

    def write(self, event: str, **fields) -> None:
        record = _json_value({"event": event, "wall_time_ns": time.time_ns(), "monotonic_ns": time.monotonic_ns(), **fields})
        line = json.dumps(record, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        with self._lock:
            if not self._file.closed:
                self._file.write(line + "\n")

    def save_observation_images(self, observation: dict) -> None:
        if self._saved_images:
            return
        for key, frame in observation.items():
            if key.startswith("observation/images/"):
                name = key.rsplit("/", 1)[-1]
                path = self.path / f"policy_{name}.png"
                if not cv2.imwrite(str(path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)):
                    raise OSError(f"Cannot save observation image: {path}")
        self._saved_images = True

    def close(self) -> None:
        self.write("session_end")
        logging.getLogger().removeHandler(self._handler)
        self._handler.close()
        with self._lock:
            self._file.close()
