"""Lightweight msgpack helpers with NumPy support."""

from __future__ import annotations

import functools

import msgpack
import numpy as np


def _pack_array(obj):
    if isinstance(obj, np.ndarray):
        if obj.dtype.kind in ("V", "O", "c"):
            raise ValueError(f"Unsupported dtype for msgpack transport: {obj.dtype}")
        return {
            b"__ndarray__": True,
            b"data": obj.tobytes(),
            b"dtype": obj.dtype.str,
            b"shape": obj.shape,
        }

    if isinstance(obj, np.generic):
        return {
            b"__npgeneric__": True,
            b"data": obj.item(),
            b"dtype": obj.dtype.str,
        }

    return obj


def _field(obj, key: str):
    if key in obj:
        return obj[key]
    raw = key.encode("utf-8")
    if raw in obj:
        return obj[raw]
    raise KeyError(key)


def _unpack_array(obj):
    # msgpack>=1.0 默认 raw=False，键会解成 str；旧包仍可能是 bytes。两边都认。
    if b"__ndarray__" in obj or "__ndarray__" in obj:
        return np.ndarray(
            buffer=_field(obj, "data"),
            dtype=np.dtype(_field(obj, "dtype")),
            shape=_field(obj, "shape"),
        )
    if b"__npgeneric__" in obj or "__npgeneric__" in obj:
        return np.dtype(_field(obj, "dtype")).type(_field(obj, "data"))
    return obj


Packer = functools.partial(msgpack.Packer, default=_pack_array)
packb = functools.partial(msgpack.packb, default=_pack_array)
unpackb = functools.partial(msgpack.unpackb, object_hook=_unpack_array)
