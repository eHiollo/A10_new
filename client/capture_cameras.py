#!/usr/bin/env python3
from __future__ import annotations

import argparse
import logging
from pathlib import Path

import cv2


def _normalize_source_token(source: str) -> tuple[int | str, str]:
    raw = source.strip()
    if raw.isdigit():
        idx = int(raw)
        return idx, f"video{idx}"
    if raw.startswith("/dev/video"):
        suffix = raw.removeprefix("/dev/video")
        if suffix.isdigit():
            return raw, f"video{suffix}"
    safe = raw.replace("/", "_").replace(" ", "_")
    return raw, safe


def _source_key_for_rotate(source: str) -> str:
    raw = source.strip()
    if raw.isdigit():
        return raw
    if raw.startswith("/dev/video"):
        return raw.removeprefix("/dev/video")
    return raw


def _capture_one(source: str, out_dir: Path, width: int, height: int, rotate_180: bool) -> bool:
    cap_source, name_token = _normalize_source_token(source)
    cap = cv2.VideoCapture(cap_source)
    if not cap.isOpened():
        logging.error("无法打开摄像头: %s", source)
        return False

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    ok, frame = cap.read()
    cap.release()

    if not ok or frame is None:
        logging.error("读取画面失败: %s", source)
        return False

    if rotate_180:
        frame = cv2.rotate(frame, cv2.ROTATE_180)

    out_path = out_dir / f"{name_token}.jpg"
    cv2.imwrite(str(out_path), frame)
    h, w = frame.shape[:2]
    logging.info("已保存: %s (分辨率: %dx%d, 来源: %s)", out_path, w, h, source)
    return True


def _auto_detect_sources(max_index: int) -> list[str]:
    sources: list[str] = []
    for i in range(max_index + 1):
        cap = cv2.VideoCapture(i)
        ok = cap.isOpened()
        cap.release()
        if ok:
            sources.append(str(i))
    return sources


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="抓取摄像头画面并按端口号命名保存图片（例如 video0.jpg / video2.jpg）。"
    )
    parser.add_argument(
        "--sources",
        nargs="*",
        default=[],
        help="摄像头端口列表，如: 0 2 或 /dev/video0 /dev/video2。不传则自动探测。",
    )
    parser.add_argument("--max-index", type=int, default=8, help="自动探测时扫描的最大端口号。")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--output-dir", type=str, default="camera_snaps")
    parser.add_argument(
        "--rotate-180-sources",
        nargs="*",
        default=["0"],
        help="保存前旋转 180° 的端口列表（数字或与 /dev/video 后缀一致），默认仅 0（顶视倒装）。传空列表可关闭: --rotate-180-sources",
    )
    return parser


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    args = build_parser().parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    sources = list(args.sources)
    if not sources:
        sources = _auto_detect_sources(args.max_index)
        logging.info("自动探测到摄像头端口: %s", ", ".join(sources) if sources else "(无)")

    if not sources:
        logging.error("未发现可用摄像头。")
        return

    rotate_keys = {_source_key_for_rotate(s) for s in (args.rotate_180_sources or [])}

    ok_count = 0
    for src in sources:
        key = _source_key_for_rotate(src)
        do_rotate = key in rotate_keys
        if _capture_one(src, out_dir, args.width, args.height, rotate_180=do_rotate):
            ok_count += 1

    logging.info("完成: 成功 %d / 总计 %d", ok_count, len(sources))


if __name__ == "__main__":
    main()
