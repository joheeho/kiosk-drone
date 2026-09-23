#!/usr/bin/env python3
"""Capture one RGB frame + one depth frame from the running gz sim x500_depth,
save them, and run ArUco detection on the RGB frame."""

import threading
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
import gz.transport13 as gz_transport

RGB_TOPIC = "/world/kiosk/model/x500_depth_0/link/camera_link/sensor/IMX214/image"
DEPTH_TOPIC = "/depth_camera"

OUT_DIR = Path(__file__).resolve().parent.parent / "outputs"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# Physical marker size, must match MARKER_MM in generate_marker.py or any
# solvePnP/estimatePoseSingleMarkers distance estimate built on this capture
# will carry a systematic bias (previously +34% from a stale 85mm assumption).
MARKER_MM = 150
EXPECTED_IDS = {0, 1, 2, 3}  # top-left, top-right, bottom-right, bottom-left

rgb_frame = {}
depth_frame = {}
rgb_event = threading.Event()
depth_event = threading.Event()


def rgb_cb(msg: Image):
    if rgb_event.is_set():
        return
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    img = arr.reshape((msg.height, msg.width, 3))  # RGB_INT8
    rgb_frame["img"] = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    rgb_event.set()


def depth_cb(msg: Image):
    if depth_event.is_set():
        return
    arr = np.frombuffer(msg.data, dtype=np.float32)
    img = arr.reshape((msg.height, msg.width))  # R_FLOAT32
    depth_frame["img"] = img.copy()
    depth_event.set()


def main():
    node = gz_transport.Node()
    node.subscribe(Image, RGB_TOPIC, rgb_cb)
    node.subscribe(Image, DEPTH_TOPIC, depth_cb)

    print("waiting for frames...")
    got_rgb = rgb_event.wait(timeout=10)
    got_depth = depth_event.wait(timeout=10)
    print(f"rgb captured: {got_rgb}, depth captured: {got_depth}")

    if not got_rgb:
        raise SystemExit("failed to capture RGB frame")

    bgr = rgb_frame["img"]
    cv2.imwrite(f"{OUT_DIR}/marker_test_frame.png", bgr)
    print(f"saved {OUT_DIR}/marker_test_frame.png  shape={bgr.shape}")

    if got_depth:
        d = depth_frame["img"]
        finite = d[np.isfinite(d)]
        print(f"depth frame shape={d.shape} min={finite.min() if finite.size else 'nan'} "
              f"max={finite.max() if finite.size else 'nan'}")
        d_vis = np.nan_to_num(d, nan=0.0, posinf=0.0)
        d_norm = cv2.normalize(d_vis, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imwrite(f"{OUT_DIR}/depth_test_frame.png", d_norm)
        print(f"saved {OUT_DIR}/depth_test_frame.png (normalized visualization)")

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    detected_ids = ids.flatten().tolist() if ids is not None else []
    print(f"marker size assumed: {MARKER_MM}mm (must match generate_marker.py MARKER_MM)")
    print(f"ArUco detected ids: {detected_ids} ({len(detected_ids)} marker(s))")

    counts = Counter(detected_ids)
    for expected_id in sorted(EXPECTED_IDS):
        c = counts.get(expected_id, 0)
        status = "DETECTED" if c else "missing"
        print(f"  id={expected_id}: {status} (count={c})")
    unexpected = [i for i in detected_ids if i not in EXPECTED_IDS]
    if unexpected:
        print(f"  unexpected ids detected: {unexpected}")

    id_to_corners = {}
    if ids is not None:
        for marker_id, corner_set in zip(detected_ids, corners):
            id_to_corners[marker_id] = corner_set.tolist()
    for marker_id in sorted(id_to_corners):
        print(f"  corners[id={marker_id}]: {id_to_corners[marker_id]}")
    if corners:
        print(f"corners[0]: {corners[0].tolist()}")

    annotated = bgr.copy()
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
    cv2.imwrite(f"{OUT_DIR}/marker_test_frame_annotated.png", annotated)
    print(f"saved {OUT_DIR}/marker_test_frame_annotated.png")

    success = bool(EXPECTED_IDS & set(detected_ids))
    print(f"DETECTION_SUCCESS={success}")


if __name__ == "__main__":
    main()
