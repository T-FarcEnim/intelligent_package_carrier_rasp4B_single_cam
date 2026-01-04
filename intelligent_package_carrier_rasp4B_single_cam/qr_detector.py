#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
qr_detector.py
二维码检测 + 距离与姿态估计模块（取消白名单和位数限制）

功能：
1）按帧扫描二维码
2）不再校验内容是否属于配置中的字典
3）不再校验位数
4）利用二维码实际尺寸 + 相机内参计算绝对距离
5）根据角点求：
    - 左右偏移
    - 上下偏移
    - 左右朝向趋势
    - 上下朝向趋势
6）只选最近二维码作为目标
7）丢失二维码时输出 lost=True

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
                 qr_config="config/qr_params.json",
                 debug_mode=False):

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
        
        # 不再使用这些校验条件
        # self.valid_digits = int(self.qr_cfg["valid_id_digits"])
        # self.valid_dict = self.qr_cfg.get("qr_dict", {})
        
        self.debug_mode = debug_mode
        self.detector = cv2.QRCodeDetector()
        
        if self.debug_mode:
            print("=" * 50)
            print("QRDetector Configuration (No Restrictions):")
            print(f"  QR Size: {self.qr_size_cm} cm")
            print(f"  Camera fx: {self.fx}, fy: {self.fy}")
            print(f"  Camera cx: {self.cx}, cy: {self.cy}")
            print("=" * 50)

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
        
        if self.debug_mode:
            print(f"[DEBUG] Pixel size: {pixel_size:.1f} px")
            print(f"[DEBUG] Estimated distance: {Z:.1f} cm")
            
        return float(Z)

    def detect(self, frame):
        """
        检测二维码，不再进行白名单和位数校验
        """
        if self.debug_mode:
            print("\n" + "=" * 30)
            print("Starting QR Detection (No Restrictions)")
            print(f"Frame shape: {frame.shape}")

        # 尝试检测二维码
        retval, data_list, pts_list, _ = self.detector.detectAndDecodeMulti(frame)
        
        if self.debug_mode:
            print(f"OpenCV detectAndDecodeMulti returned: {retval}")
            print(f"Number of detected codes: {len(data_list) if data_list else 0}")

        if not retval or pts_list is None or len(data_list) == 0:
            if self.debug_mode:
                print("No QR codes found in frame")
            return {"lost": True}

        best_target = None
        best_dist = 1e9
        
        if self.debug_mode:
            print(f"Found {len(data_list)} QR code(s)")

        for i, (data, pts) in enumerate(zip(data_list, pts_list)):
            if self.debug_mode:
                print(f"\nQR Candidate #{i+1}:")
                print(f"  Raw Data: '{data}'")
                print(f"  Points: {pts}")

            if not data:
                if self.debug_mode:
                    print("  → Skipped: Empty data")
                continue

            pts_array = np.array(pts, dtype=np.float32)
            
            # 检查二维码尺寸是否合理
            p1, p2, p3, p4 = pts_array
            width1 = np.linalg.norm(p2 - p1)
            width2 = np.linalg.norm(p3 - p4)
            pixel_size = (width1 + width2) / 2.0
            
            if self.debug_mode:
                print(f"  Pixel size: {pixel_size:.1f}")

            # 如果二维码太小，可能不可靠
            if pixel_size < 20:  # 20像素是较低阈值
                if self.debug_mode:
                    print(f"  → Skipped: Too small ({pixel_size:.1f} px < 20 px)")
                continue

            dist = self._estimate_distance(pts_array)
            if dist is None:
                if self.debug_mode:
                    print("  → Skipped: Could not estimate distance")
                continue

            if self.debug_mode:
                print(f"  Distance: {dist:.1f} cm")

            if dist < best_dist:
                best_dist = dist
                best_target = (data, pts_array, dist)
                if self.debug_mode:
                    print(f"  → New best candidate!")

        if best_target is None:
            if self.debug_mode:
                print("\nNo valid QR codes found after filtering by size")
            return {"lost": True}

        data, pts, dist = best_target
        offsets = self._compute_offsets(pts)

        if self.debug_mode:
            print(f"\n✓ Selected QR Code:")
            print(f"  ID: {data}")
            print(f"  Distance: {dist:.1f} cm")
            print(f"  Center Offset: x={offsets['cx_off']:.1f}, y={offsets['cy_off']:.1f}")

        return dict(
            lost=False,
            data=data,
            distance_cm=dist,
            offsets=offsets,
            points=pts.tolist()
        )

    def _compute_offsets(self, pts):
        """
        计算偏移与朝向趋势
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


if __name__ == "__main__":
    # 测试模式
    detector = QRDetector(debug_mode=True)
    print("QRDetector initialized with debug mode (no restrictions)")