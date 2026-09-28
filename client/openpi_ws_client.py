from __future__ import annotations

import logging
import time
import urllib.error
import urllib.request
from typing import Any

import websockets.sync.client

from msgpack_numpy import Packer, unpackb

logger = logging.getLogger(__name__)


class OpenPIWebsocketClient:
    """OpenPI websocket client with reconnect support.

    协议：msgpack-numpy，WebSocket 关闭压缩。连上后先收一帧 metadata，
    之后每次 ``infer`` 发一帧 observation、收一帧结果。回包若是字符串则是报错堆栈。
    """

    def __init__(
        self,
        host: str,
        port: int,
        api_key: str | None = None,
        reconnect_interval_s: float = 2.0,
    ) -> None:
        self._host = host
        self._port = port
        self._uri = f"ws://{host}:{port}"
        self._healthz_url = f"http://{host}:{port}/healthz"
        self._api_key = api_key
        self._reconnect_interval_s = reconnect_interval_s
        self._conn: websockets.sync.client.ClientConnection | None = None
        self._packer = Packer()
        self._server_metadata: dict[str, Any] = {}
        self._connect()

    @property
    def server_metadata(self) -> dict[str, Any]:
        return dict(self._server_metadata)

    def _check_healthz(self) -> None:
        request = urllib.request.Request(self._healthz_url, method="GET")
        try:
            with urllib.request.urlopen(request, timeout=2.0) as response:
                body = response.read().decode("utf-8", errors="replace").strip()
                status = getattr(response, "status", 200)
        except urllib.error.URLError as exc:
            raise RuntimeError(f"healthz {self._healthz_url} failed: {exc}") from exc
        if status != 200 or body != "OK":
            raise RuntimeError(f"healthz {self._healthz_url} -> {status} {body!r}")

    def _connect(self) -> None:
        while True:
            try:
                self._check_healthz()
                headers = {"Authorization": f"Api-Key {self._api_key}"} if self._api_key else None
                # compression=None：服务端约定不要开 WebSocket 压缩。
                self._conn = websockets.sync.client.connect(
                    self._uri,
                    compression=None,
                    max_size=None,
                    additional_headers=headers,
                )
                metadata_msg = self._conn.recv()
                if isinstance(metadata_msg, str):
                    raise RuntimeError(f"Expected metadata as bytes, got text: {metadata_msg}")
                metadata = unpackb(metadata_msg)
                if not isinstance(metadata, dict):
                    raise RuntimeError(f"Expected metadata dict, got {type(metadata).__name__}")
                self._server_metadata = metadata
                logger.info("Connected to OpenPI websocket server: %s metadata=%s", self._uri, metadata)
                return
            except Exception as exc:  # noqa: BLE001
                logger.warning("OpenPI websocket connect failed (%s), retrying...", exc)
                time.sleep(self._reconnect_interval_s)

    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        if self._conn is None:
            self._connect()

        assert self._conn is not None
        try:
            payload = self._packer.pack(observation)
            self._conn.send(payload)
            response = self._conn.recv()
            if isinstance(response, str):
                raise RuntimeError(f"Inference server returned error text: {response}")
            return unpackb(response)
        except Exception:  # noqa: BLE001
            logger.exception("OpenPI websocket inference failed; reconnecting.")
            self.close()
            self._connect()
            raise

    def close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:  # noqa: BLE001
                pass
            self._conn = None
