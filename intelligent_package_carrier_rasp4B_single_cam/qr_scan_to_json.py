#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
qr_scan_to_json.py
独立二维码扫描工具 —— 输出识别结果 JSON

用途：
- 采集目标二维码参数
- 标定二维码真实尺寸与像素关系
- 生成可供主系统读取的二维码样本文件

特点：
- 不开启摄像头预览
- 不参与运动/避障
- 读取一次成功即保存并退出
"""

import json
import time
from pathlib import Path
import cv2
import numpy as np

from camera_module import CameraModule


OUTPUT_FILE = "qr_scan_result.json"


def detect_qr(frame):
    detector = cv2.QRCodeDetector()
    data, points, _ = detector.detectAndDecode(frame)

    if not data or points is None:
        return None

    pts = points.reshape(-1, 2)
    cx = float(np.mean(pts[:, 0]))
    cy = float(np.mean(pts[:, 1]))

    # 近似二维码像素边长
    side = float(np.linalg.norm(pts[0] - pts[1]))

    return {
        "content": data,
        "corners": pts.tolist(),
        "center_px": [cx, cy],
        "pixel_size": side,
        "timestamp": time.time()
    }


def save_json(result, path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

def update_qr_dict(qr_id, scan_data):
    with open("qr_params.json", "r") as f:
        params = json.load(f)
    
    params["qr_dict"][qr_id] = {
        "calibration_data": scan_data,
        "added_time": time.strftime("%Y-%m-%d %H:%M:%S")
    }
    
    with open("qr_params.json", "w") as f:
        json.dump(params, f, indent=2, ensure_ascii=False)

def main():
    cam = CameraModule()
    cam.open()

    print("二维码扫描工具启动，正在等待识别...（按 Ctrl+C 退出）")

    try:
        for frame in cam.frames():
            result = detect_qr(frame)
            if result is None:
                continue

            save_json(result, OUTPUT_FILE)

            print(f"识别成功，写入文件：{OUTPUT_FILE}")
            print(json.dumps(result, indent=2, ensure_ascii=False))
            break

    finally:
        cam.close()


if __name__ == "__main__":
    main()