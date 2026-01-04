#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
realtime_qr_preview.py
实时预览二维码识别情况

功能：
1. 实时显示摄像头画面
2. 实时检测并标记二维码
3. 显示检测结果和距离信息
4. 显示帧率和状态信息

按键说明：
- 按 'q' 或 'ESC' 退出程序
- 按 's' 保存当前帧为 'capture.jpg'
- 按 'd' 切换调试信息显示
"""

import cv2
import time
import numpy as np
from camera_module import CameraModule
from qr_detector import QRDetector


class QRCodeRealtimePreview:
    def __init__(self, camera_config="config/camera_intrinsics.json", 
                 qr_config="config/qr_params.json", target_fps=15):
        # 初始化摄像头模块
        self.camera = CameraModule(camera_config, target_fps)
        self.camera.open()
        
        # 初始化二维码检测器
        self.detector = QRDetector(camera_config, qr_config)
        
        # 状态变量
        self.running = True
        self.show_debug = False
        self.frame_count = 0
        self.fps = 0
        self.last_time = time.time()
        
        # 颜色定义
        self.COLOR_RED = (0, 0, 255)
        self.COLOR_GREEN = (0, 255, 0)
        self.COLOR_BLUE = (255, 0, 0)
        self.COLOR_YELLOW = (0, 255, 255)
        self.COLOR_PURPLE = (255, 0, 255)
        self.COLOR_CYAN = (255, 255, 0)
        self.COLOR_WHITE = (255, 255, 255)
        
        print("QR Code Real-time Preview")
        print("=" * 40)
        print("Controls:")
        print("  'q' or ESC: Exit")
        print("  's': Save current frame")
        print("  'd': Toggle debug info")
        print("=" * 40)
    
    def calculate_fps(self):
        """计算并更新FPS"""
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.last_time
        
        if elapsed >= 1.0:  # 每秒更新一次FPS
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_time = current_time
    
    def draw_info_panel(self, frame, result):
        """在图像上绘制信息面板"""
        h, w = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_small = cv2.FONT_HERSHEY_PLAIN
        
        # 创建半透明背景板
        info_panel = np.zeros((180, 400, 3), dtype=np.uint8)
        info_panel[:] = (40, 40, 40)  # 深灰色背景
        
        # 添加标题
        cv2.putText(info_panel, "QR Code Detection Status", (10, 30), 
                   font, 0.7, self.COLOR_CYAN, 1)
        
        # 添加分隔线
        cv2.line(info_panel, (10, 40), (390, 40), self.COLOR_WHITE, 1)
        
        y_offset = 65
        
        # 显示检测状态
        if result["lost"]:
            status_text = "No QR Code Detected"
            status_color = self.COLOR_RED
        else:
            status_text = "QR Code Detected"
            status_color = self.COLOR_GREEN
        
        cv2.putText(info_panel, f"Status: {status_text}", (10, y_offset), 
                   font, 0.5, status_color, 1)
        y_offset += 25
        
        if not result["lost"]:
            # 显示二维码信息
            cv2.putText(info_panel, f"QR ID: {result['data']}", (10, y_offset), 
                       font, 0.5, self.COLOR_YELLOW, 1)
            y_offset += 25
            
            cv2.putText(info_panel, f"Distance: {result['distance_cm']:.1f} cm", (10, y_offset), 
                       font, 0.5, self.COLOR_YELLOW, 1)
            y_offset += 25
            
            # 显示偏移信息
            offsets = result["offsets"]
            cx_off = offsets["cx_off"]
            cy_off = offsets["cy_off"]
            
            # 判断左右位置
            if cx_off > 20:
                x_pos = "Right"
                x_color = self.COLOR_RED
            elif cx_off < -20:
                x_pos = "Left"
                x_color = self.COLOR_RED
            else:
                x_pos = "Center"
                x_color = self.COLOR_GREEN
            
            # 判断上下位置
            if cy_off > 20:
                y_pos = "Below"
                y_color = self.COLOR_RED
            elif cy_off < -20:
                y_pos = "Above"
                y_color = self.COLOR_RED
            else:
                y_pos = "Center"
                y_color = self.COLOR_GREEN
            
            cv2.putText(info_panel, f"Horizontal: {x_pos} ({cx_off:.0f} px)", (10, y_offset), 
                       font, 0.5, x_color, 1)
            y_offset += 25
            
            cv2.putText(info_panel, f"Vertical: {y_pos} ({cy_off:.0f} px)", (10, y_offset), 
                       font, 0.5, y_color, 1)
            y_offset += 25
        
        # 显示FPS
        cv2.putText(info_panel, f"FPS: {self.fps:.1f}", (10, y_offset), 
                   font, 0.5, self.COLOR_CYAN, 1)
        y_offset += 25
        
        # 显示调试信息（如果启用）
        if self.show_debug:
            cv2.putText(info_panel, "DEBUG MODE", (10, y_offset), 
                       font, 0.5, self.COLOR_PURPLE, 1)
            y_offset += 20
            
            # 在调试模式下，显示更多技术信息
            if not result["lost"]:
                offsets = result["offsets"]
                cv2.putText(info_panel, f"Yaw: {offsets['yaw_trend']:.1f}", (10, y_offset), 
                           font_small, 0.6, self.COLOR_WHITE, 1)
                y_offset += 15
                cv2.putText(info_panel, f"Pitch: {offsets['pitch_trend']:.1f}", (10, y_offset), 
                           font_small, 0.6, self.COLOR_WHITE, 1)
        
        # 将信息面板叠加到原图上
        panel_h, panel_w = info_panel.shape[:2]
        frame[10:10+panel_h, 10:10+panel_w] = cv2.addWeighted(
            frame[10:10+panel_h, 10:10+panel_w], 0.3, info_panel, 0.7, 0)
    
    def draw_detection_markers(self, frame, result):
        """在图像上绘制检测标记"""
        if result["lost"]:
            # 如果没有检测到二维码，在图像中心显示红色十字和文字
            h, w = frame.shape[:2]
            center_x, center_y = w // 2, h // 2
            
            # 绘制十字准星
            cv2.line(frame, (center_x - 30, center_y), (center_x - 10, center_y), 
                    self.COLOR_RED, 2)
            cv2.line(frame, (center_x + 10, center_y), (center_x + 30, center_y), 
                    self.COLOR_RED, 2)
            cv2.line(frame, (center_x, center_y - 30), (center_x, center_y - 10), 
                    self.COLOR_RED, 2)
            cv2.line(frame, (center_x, center_y + 10), (center_x, center_y + 30), 
                    self.COLOR_RED, 2)
            
            # 绘制文字
            cv2.putText(frame, "SEARCHING", (center_x - 60, center_y - 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.COLOR_RED, 2)
        else:
            # 如果检测到二维码，绘制标记
            pts = np.array(result["points"], dtype=np.int32)
            
            # 绘制绿色边界框
            cv2.polylines(frame, [pts], isClosed=True, color=self.COLOR_GREEN, thickness=2)
            
            # 绘制紫色角点
            for i, point in enumerate(pts):
                # 角点
                cv2.circle(frame, tuple(point), 6, self.COLOR_PURPLE, -1)
                cv2.circle(frame, tuple(point), 10, self.COLOR_PURPLE, 2)
                
                # 角点编号
                cv2.putText(frame, str(i+1), (point[0]+8, point[1]+8), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.COLOR_WHITE, 2)
                
                # 连线
                if i < len(pts) - 1:
                    cv2.line(frame, tuple(point), tuple(pts[i+1]), self.COLOR_PURPLE, 2)
                else:
                    cv2.line(frame, tuple(point), tuple(pts[0]), self.COLOR_PURPLE, 2)
            
            # 绘制二维码中心点
            center = pts.mean(axis=0).astype(int)
            cv2.circle(frame, tuple(center), 8, self.COLOR_YELLOW, -1)
            cv2.circle(frame, tuple(center), 12, self.COLOR_YELLOW, 2)
            
            # 绘制到图像中心的连线（显示偏移方向）
            h, w = frame.shape[:2]
            img_center = (w // 2, h // 2)
            cv2.line(frame, tuple(center), img_center, self.COLOR_CYAN, 2)
            
            # 在二维码上方显示距离
            text_pos = (pts[0][0], pts[0][1] - 20)
            cv2.putText(frame, f"{result['distance_cm']:.1f}cm", text_pos, 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.COLOR_YELLOW, 2)
    
    def save_current_frame(self, frame):
        """保存当前帧"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"qr_capture_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Frame saved as: {filename}")
        return filename
    
    def run(self):
        """主运行循环"""
        print("Starting real-time QR code detection...")
        
        try:
            for frame in self.camera.frames():
                if not self.running:
                    break
                
                # 计算FPS
                self.calculate_fps()
                
                # 检测二维码
                result = self.detector.detect(frame)
                
                # 绘制检测标记
                self.draw_detection_markers(frame, result)
                
                # 绘制信息面板
                self.draw_info_panel(frame, result)
                
                # 显示帧
                cv2.imshow("QR Code Detection - Realtime Preview", frame)
                
                # 处理按键事件
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 'q' 或 ESC
                    print("Exit requested")
                    break
                elif key == ord('s'):  # 保存当前帧
                    self.save_current_frame(frame)
                elif key == ord('d'):  # 切换调试模式
                    self.show_debug = not self.show_debug
                    debug_status = "ON" if self.show_debug else "OFF"
                    print(f"Debug mode: {debug_status}")
        
        except KeyboardInterrupt:
            print("\nProgram interrupted by user")
        
        finally:
            # 清理资源
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        self.camera.close()
        cv2.destroyAllWindows()
        print("Cleanup completed. Resources released.")


if __name__ == "__main__":
    # 创建并运行实时预览
    preview = QRCodeRealtimePreview()
    preview.run()