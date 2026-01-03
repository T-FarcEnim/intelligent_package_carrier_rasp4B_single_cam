#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
track_main.py
自动循迹—避障智能小车主控程序

功能：
1）读取摄像头帧
2）二维码识别 + 距离 + 姿态估计
3）超声波避障状态查询
4）实时向量演算 + PID 模拟
5）输出左右轮速度

设计原则：
- 避障优先
- 最近二维码为目标
- 丢失二维码 → 保持最近参数并等待恢复
- 不开启摄像头预览窗口
"""

import time

from camera_module import CameraModule
from qr_detector import QRDetector
from obstacle_sensor import ObstacleSensor
from motion_controller import MotionController


# ========== 占位电机驱动接口（部署时替换） ==========
def drive_motor(left, right):
    print(f"[DRIVE] L={left:.2f}  R={right:.2f}")


class TrackSystem:
    def __init__(self):
        self.camera = CameraModule()
        self.qr = QRDetector()
        self.obstacle = ObstacleSensor()
        self.motion = MotionController()

        self.last_valid_qr = None

    def run(self):
        self.camera.open()

        try:
            for frame in self.camera.frames():

                # ---- 避障状态 ----
                obst_state = self.obstacle.update()

                # ---- 二维码识别 ----
                qr_state = self.qr.detect(frame)

                # 丢失时保持最近目标参数
                if qr_state.get("lost", False):
                    if self.last_valid_qr is not None:
                        qr_state = self.last_valid_qr
                        qr_state["lost"] = True
                else:
                    self.last_valid_qr = qr_state

                # ---- 运动计算 ----
                motion = self.motion.compute(qr_state, obst_state)

                # ---- 电机输出 ----
                drive_motor(motion["left_speed"], motion["right_speed"])

                # 控制循环频率（与摄像头 FPS 解耦）
                time.sleep(0.02)

        finally:
            self.camera.close()


if __name__ == "__main__":
    system = TrackSystem()
    print("自动循迹系统启动...")
    system.run()