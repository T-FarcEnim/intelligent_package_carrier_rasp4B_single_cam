 #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
qr_detector.py
二维码检测 + 距离与姿态估计模块

功能：
1）按帧扫描二维码
2）校验内容是否属于配置中的字典
3）利用二维码实际尺寸 + 相机内参计算绝对距离
4）根据角点求：
    - 左右偏移
    - 上下偏移
    - 左右朝向趋势
    - 上下朝向趋势
5）只选最近二维码作为目标
6）丢失二维码时输出 lost=True

依赖：
- OpenCV
- config_loader.py
"""

import cv2
import numpy as np
from config_loader import ConfigLoader


class QRDetector:
    def __init__(self,
                 camera_config="config/camera_intrinsics.json",
                 qr_config="config/qr_params.json"):

        loader = ConfigLoader(
            camera_json=camera_config,
            qr_json=qr_config
        )

        cam = loader.load_camera_params()
        self.qr_cfg = loader.load_qr_params()

        self.fx = cam["fx"]
        self.fy = cam["fy"]
        self.cx = cam["cx"]
        self.cy = cam["cy"]

        # 二维码真实尺寸（cm）
        self.qr_size_cm = float(self.qr_cfg["qr_size_cm"])
        self.valid_digits = int(self.qr_cfg["valid_id_digits"])
        self.valid_dict = self.qr_cfg.get("qr_dict", {})

        self.detector = cv2.QRCodeDetector()

    def _estimate_distance(self, pts):
        """
        简单利用二维码外接宽度估算距离：
        Z = real_size * fx / pixel_size
        """
        p1, p2, p3, p4 = pts

        # 边长像素长度
        width1 = np.linalg.norm(p2 - p1)
        width2 = np.linalg.norm(p3 - p4)
        pixel_size = (width1 + width2) / 2.0

        if pixel_size < 1e-5:
            return None

        # cm 单位距离
        Z = (self.qr_size_cm * self.fx) / pixel_size
        return float(Z)

    def _compute_offsets(self, pts):
        """
        计算偏移与朝向趋势
        返回：
        {
            "cx_off": ...,
            "cy_off": ...,
            "yaw_trend": ...,
            "pitch_trend": ...
        }
        """

        center = pts.mean(axis=0)

        cx_off = float(center[0] - self.cx)
        cy_off = float(center[1] - self.cy)

        # 左右倾斜（简单用边斜率近似）
        left_vec = pts[0] - pts[3]
        right_vec = pts[1] - pts[2]
        yaw_trend = float((left_vec[1] - right_vec[1]) / 2.0)

        # 上下仰俯趋势
        top_y = (pts[0][1] + pts[1][1]) / 2.0
        bottom_y = (pts[2][1] + pts[3][1]) / 2.0
        pitch_trend = float(bottom_y - top_y)

        return dict(
            cx_off=cx_off,
            cy_off=cy_off,
            yaw_trend=yaw_trend,
            pitch_trend=pitch_trend
        )

    def detect(self, frame):
        """
        输入：一帧图像
        输出：
        {
            "lost": bool,
            "data": str,
            "distance_cm": float,
            "offsets": {...},
            "points": [[x,y], ...]
        }
        """

        data_list, pts_list, _ = self.detector.detectAndDecodeMulti(frame)

        if pts_list is None or len(data_list) == 0:
            return {"lost": True}

        best_target = None
        best_dist = 1e9

        for data, pts in zip(data_list, pts_list):
            if not data:
                continue

            # 校验内容是否符合规则
            if not data.isdigit() or len(data) != self.valid_digits:
                continue

            if self.valid_dict and data not in self.valid_dict:
                continue

            pts = np.array(pts, dtype=np.float32)

            dist = self._estimate_distance(pts)
            if dist is None:
                continue

            if dist < best_dist:
                best_dist = dist
                best_target = (data, pts, dist)

        if best_target is None:
            return {"lost": True}

        data, pts, dist = best_target
        offsets = self._compute_offsets(pts)

        return dict(
            lost=False,
            data=data,
            distance_cm=dist,
            offsets=offsets,
            points=pts.tolist()
        )


if __name__ == "__main__":
    # 简单自检（不显示图像）
    print("qr_detector 模块加载成功，可与 camera_module 配合使用")