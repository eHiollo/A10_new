from __future__ import annotations

import logging
import time
from typing import Any

import websockets.sync.client

from msgpack_numpy import Packer, unpackb

logger = logging.getLogger(__name__)


class OpenPIWebsocketClient:
    """OpenPI websocket client with reconnect support."""

    def __init__(
        self,
        host: str,
        port: int,
        api_key: str | None = None,
        reconnect_interval_s: float = 2.0,
    ) -> None:
        self._uri = f"ws://{host}:{port}"
        self._api_key = api_key
        self._reconnect_interval_s = reconnect_interval_s
        self._conn: websockets.sync.client.ClientConnection | None = None
        self._packer = Packer()
        self._server_metadata: dict[str, Any] = {}
        self._connect()

    @property
    def server_metadata(self) -> dict[str, Any]:
        return dict(self._server_metadata)

    def _connect(self) -> None:
        while True:
            try:
                headers = {"Authorization": f"Api-Key {self._api_key}"} if self._api_key else None
                self._conn = websockets.sync.client.connect(
                    self._uri,
                    compression=None,
                    max_size=None,
                    additional_headers=headers,
                )
                metadata_msg = self._conn.recv()
                if isinstance(metadata_msg, str):
                    raise RuntimeError(f"Expected metadata as bytes, got text: {metadata_msg}")
                self._server_metadata = unpackb(metadata_msg)
                logger.info("Connected to OpenPI websocket server: %s", self._uri)
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
