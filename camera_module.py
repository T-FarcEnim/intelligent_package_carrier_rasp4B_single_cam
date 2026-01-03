#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
camera_module.py
树莓派摄像头采集模块（无预览窗口）

功能：
1）读取相机内参 json
2）启动摄像头并以固定帧率输出图像
3）进行简单去畸变
4）提供迭代接口给其他模块调用

依赖：
- OpenCV
- config_loader.py
"""

import cv2
import time
import numpy as np
from config_loader import ConfigLoader


class CameraModule:
    def __init__(self, config_path="config/camera_intrinsics.json", target_fps=15):
        loader = ConfigLoader(camera_json=config_path)
        self.cam_cfg = loader.load_camera_params()

        self.fx = self.cam_cfg["fx"]
        self.fy = self.cam_cfg["fy"]
        self.cx = self.cam_cfg["cx"]
        self.cy = self.cam_cfg["cy"]
        self.dist = np.array(self.cam_cfg["dist"], dtype=np.float32)

        self.K = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0,    0,     1]
        ], dtype=np.float32)

        self.cap = None
        self.target_fps = max(5, target_fps)
        self.frame_interval = 1.0 / self.target_fps

    def open(self, device_id=0, width=640, height=480):
        self.cap = cv2.VideoCapture(device_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)

        if not self.cap.isOpened():
            raise RuntimeError("摄像头打开失败")

    def undistort(self, frame):
        h, w = frame.shape[:2]
        new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (w, h), 1)
        return cv2.undistort(frame, self.K, self.dist, None, new_K)

    def frames(self):
        """
        生成器：持续输出去畸变帧
        """
        if self.cap is None:
            self.open()

        last_time = 0

        while True:
            ok, frame = self.cap.read()
            if not ok:
                continue

            now = time.time()
            if now - last_time < self.frame_interval:
                continue
            last_time = now

            frame = self.undistort(frame)
            yield frame

    def close(self):
        if self.cap:
            self.cap.release()
            self.cap = None


if __name__ == "__main__":
    """
    仅测试采集是否正常（不显示）
    在终端打印帧尺寸
    """
    cam = CameraModule()
    cam.open()

    try:
        for i, f in enumerate(cam.frames()):
            print("收到帧", f.shape)
            if i > 10:
                break
    finally:
        cam.close()