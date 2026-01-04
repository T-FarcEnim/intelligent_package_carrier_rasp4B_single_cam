#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
camera_module.py
树莓派摄像头采集模块（高帧率优化版）

优化措施：
1）使用V4L2后端提高性能
2）设置MJPG编码减少带宽
3）降低分辨率提高帧率
4）优化曝光和增益设置
5）使用多线程缓冲
6）可选关闭去畸变提高帧率
"""

import cv2
import time
import numpy as np
import threading
from queue import Queue
from config_loader import ConfigLoader


class CameraModule:
    def __init__(self, config_path="config/camera_intrinsics.json", 
                 target_fps=30, enable_undistort=False):
        """
        初始化摄像头模块
        
        Args:
            target_fps: 目标帧率，默认30fps
            enable_undistort: 是否启用去畸变（会降低帧率）
        """
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
        self.target_fps = min(max(5, target_fps), 60)  # 限制在5-60fps之间
        self.frame_interval = 1.0 / self.target_fps
        self.enable_undistort = enable_undistort
        self.last_frame_time = 0
        self.frame_count = 0
        self.actual_fps = 0
        
        # 多线程缓冲
        self.frame_queue = Queue(maxsize=2)  # 小缓冲减少延迟
        self.capture_thread = None
        self.running = False
        
        # 高帧率优化参数
        self.camera_settings = {
            'exposure_time': -6,          # 较短曝光，适合运动场景
            'contrast': 70,               # 中等对比度
            'saturation': 60,             # 中等饱和度
            'brightness': 50,             # 中等亮度
            'gain': 15,                   # 较高增益补偿短曝光
            'white_balance_auto': 0,      # 关闭自动白平衡
            'white_balance': 4500,        # 固定白平衡
            'sharpness': 60,              # 中等锐化
            'backlight_comp': 0,          # 关闭背光补偿
            'auto_exposure': 1,           # 自动曝光模式
            'exposure_auto_priority': 0,  # 曝光优先级，0=速度优先
            'power_line_freq': 1,         # 50Hz防闪烁（中国）
            'focus_auto': 0,              # 关闭自动对焦
            'focus_absolute': 0,          # 固定焦距
            'buffersize': 1,              # 减少缓冲区大小
        }
        
        # MJPG编码设置（更高压缩比，适合高帧率）
        self.codec = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        
        print(f"摄像头模块初始化 - 目标帧率: {self.target_fps}fps, 去畸变: {enable_undistort}")

    def open(self, device_id=0, width=640, height=480):
        """
        打开摄像头并设置优化参数
        使用较低分辨率提高帧率
        """
        try:
            # 优先尝试V4L2后端，性能更好
            self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
            
            if not self.cap.isOpened():
                print("V4L2后端打开失败，尝试默认后端...")
                self.cap = cv2.VideoCapture(device_id)
            
            # 设置MJPG编码
            self.cap.set(cv2.CAP_PROP_FOURCC, self.codec)
            
            # 设置分辨率（降低分辨率提高帧率）
            width = 640  # 固定为640x480或更低
            height = 480
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            # 设置目标帧率
            self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)
            
            # 应用摄像头优化参数
            self._apply_camera_settings()
            
            # 设置缓冲区大小为1，减少延迟
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            if not self.cap.isOpened():
                raise RuntimeError("摄像头打开失败，请检查连接")
            
            # 验证实际设置
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            print(f"摄像头已打开 - 分辨率: {actual_width}x{actual_height}, 帧率: {actual_fps:.1f}fps")
            
            # 启动采集线程
            self.running = True
            self.capture_thread = threading.Thread(target=self._capture_frames, daemon=True)
            self.capture_thread.start()
            
            time.sleep(0.5)  # 给摄像头启动时间
            
        except Exception as e:
            print(f"摄像头打开错误: {e}")
            if self.cap:
                self.cap.release()
            raise

    def _apply_camera_settings(self):
        """应用摄像头优化参数（高帧率优化版）"""
        if self.cap is None:
            return
            
        try:
            # 使用V4L2特定参数（如果可用）
            # 1. 曝光设置
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1=手动模式
            
            # 尝试设置曝光时间（不同摄像头参数不同）
            for prop_id in [cv2.CAP_PROP_EXPOSURE, cv2.CAP_PROP_EXPOSURE_ABSOLUTE]:
                try:
                    self.cap.set(prop_id, self.camera_settings['exposure_time'])
                    break
                except:
                    continue
            
            # 2. 关闭自动白平衡
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            
            # 3. 增益设置
            self.cap.set(cv2.CAP_PROP_GAIN, self.camera_settings['gain'])
            
            # 4. 对比度和饱和度
            self.cap.set(cv2.CAP_PROP_CONTRAST, self.camera_settings['contrast'] / 100.0 * 255)
            self.cap.set(cv2.CAP_PROP_SATURATION, self.camera_settings['saturation'] / 100.0 * 255)
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.camera_settings['brightness'] / 100.0 * 255)
            
            # 5. 锐度
            self.cap.set(cv2.CAP_PROP_SHARPNESS, self.camera_settings['sharpness'] / 100.0 * 255)
            
            # 6. 关闭自动对焦
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            
            # 7. 防闪烁（根据地区调整）
            try:
                self.cap.set(cv2.CAP_PROP_XI_ACQ_TIMING_MODE, 2)  # 某些摄像头的防闪烁模式
            except:
                pass
                
        except Exception as e:
            print(f"部分摄像头参数设置失败: {e}")
            # 继续执行，即使部分参数设置失败

    def _capture_frames(self):
        """采集线程函数"""
        print("开始采集帧...")
        last_fps_time = time.time()
        
        while self.running:
            try:
                # 读取帧
                ret, frame = self.cap.read()
                if not ret:
                    print("采集失败，尝试重新初始化...")
                    time.sleep(0.1)
                    continue
                
                # 简单的去畸变（如果需要）
                if self.enable_undistort:
                    h, w = frame.shape[:2]
                    new_K = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (w, h), 1)[0]
                    frame = cv2.undistort(frame, self.K, self.dist, None, new_K)
                
                # 放入队列（如果队列满则丢弃旧帧）
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()  # 丢弃最旧的帧
                    except:
                        pass
                
                self.frame_queue.put(frame)
                
                # 计算实际FPS
                self.frame_count += 1
                current_time = time.time()
                if current_time - last_fps_time >= 1.0:  # 每秒更新一次
                    self.actual_fps = self.frame_count
                    self.frame_count = 0
                    last_fps_time = current_time
                    
                    if self.frame_count % 100 == 0:  # 每100帧打印一次
                        print(f"采集线程 - 实际FPS: {self.actual_fps}")
                
            except Exception as e:
                print(f"采集线程错误: {e}")
                time.sleep(0.01)

    def fast_undistort(self, frame):
        """快速去畸变（简化版，只做必要的校正）"""
        if not self.enable_undistort:
            return frame
            
        # 使用简化校正，而不是完整的getOptimalNewCameraMatrix
        h, w = frame.shape[:2]
        try:
            # 预计算校正映射（一次性计算）
            if not hasattr(self, 'map1') or not hasattr(self, 'map2'):
                self.map1, self.map2 = cv2.initUndistortRectifyMap(
                    self.K, self.dist, None, self.K, (w, h), cv2.CV_32FC1)
            
            # 使用预计算的映射快速校正
            return cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
        except:
            # 如果快速校正失败，返回原图
            return frame

    def frames(self, skip_frames=0, timeout=1.0):
        """
        生成器：持续输出帧
        
        Args:
            skip_frames: 跳过的帧数（用于进一步降低处理频率）
            timeout: 获取帧的超时时间（秒）
        """
        if self.cap is None or not self.running:
            self.open()
        
        skip_counter = 0
        last_print_time = time.time()
        
        while self.running:
            try:
                # 从队列获取帧，带超时
                frame = self.frame_queue.get(timeout=timeout)
                
                # 应用跳帧策略
                skip_counter += 1
                if skip_frames > 0 and skip_counter <= skip_frames:
                    continue
                skip_counter = 0
                
                # 快速去畸变
                frame = self.fast_undistort(frame)
                
                yield frame
                
                # 定期打印状态
                current_time = time.time()
                if current_time - last_print_time >= 2.0:
                    queue_size = self.frame_queue.qsize()
                    print(f"帧生成器 - 队列大小: {queue_size}, 实际FPS: {self.actual_fps}")
                    last_print_time = current_time
                    
            except Exception as e:
                print(f"获取帧错误: {e}")
                time.sleep(0.01)

    def get_frame(self, timeout=0.1):
        """获取单帧（非阻塞）"""
        try:
            return self.frame_queue.get(timeout=timeout)
        except:
            return None

    def set_resolution(self, width, height):
        """动态调整分辨率（可能需要重启摄像头）"""
        if self.cap:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            print(f"分辨率设置为: {width}x{height}")

    def adjust_for_fps(self, desired_fps):
        """根据目标帧率调整参数"""
        self.target_fps = min(max(5, desired_fps), 60)
        
        # 根据帧率调整曝光和增益
        if desired_fps >= 30:
            # 高帧率模式
            self.camera_settings['exposure_time'] = -8
            self.camera_settings['gain'] = 20
            print(f"切换到高帧率模式 ({desired_fps}fps)")
        else:
            # 标准帧率模式
            self.camera_settings['exposure_time'] = -5
            self.camera_settings['gain'] = 10
            print(f"切换到标准帧率模式 ({desired_fps}fps)")
        
        # 重新应用设置
        if self.cap:
            self._apply_camera_settings()

    def get_performance_info(self):
        """获取性能信息"""
        queue_size = self.frame_queue.qsize() if hasattr(self, 'frame_queue') else 0
        return {
            'target_fps': self.target_fps,
            'actual_fps': self.actual_fps,
            'queue_size': queue_size,
            'enable_undistort': self.enable_undistort,
            'frame_count': self.frame_count
        }

    def close(self):
        """关闭摄像头"""
        print("正在关闭摄像头...")
        self.running = False
        
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        
        if self.cap:
            self.cap.release()
            self.cap = None
            
        # 清空队列
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except:
                break
                
        print("摄像头已关闭")


if __name__ == "__main__":
    """
    测试高帧率摄像头
    """
    print("=== 高帧率摄像头测试 ===")
    
    # 测试不同配置
    configs = [
        {"enable_undistort": False, "target_fps": 30},
        {"enable_undistort": True, "target_fps": 20},
    ]
    
    for config in configs:
        print(f"\n测试配置: {config}")
        
        cam = CameraModule(
            enable_undistort=config["enable_undistort"],
            target_fps=config["target_fps"]
        )
        
        try:
            cam.open()
            
            # 采集100帧测试性能
            start_time = time.time()
            frame_count = 0
            
            for frame in cam.frames():
                frame_count += 1
                
                if frame_count % 50 == 0:
                    elapsed = time.time() - start_time
                    current_fps = frame_count / elapsed if elapsed > 0 else 0
                    print(f"  已采集 {frame_count} 帧, 当前FPS: {current_fps:.1f}")
                
                if frame_count >= 100:
                    break
            
            elapsed = time.time() - start_time
            avg_fps = frame_count / elapsed if elapsed > 0 else 0
            print(f"  平均FPS: {avg_fps:.1f}")
            
        finally:
            cam.close()
            time.sleep(0.5)  # 等待摄像头完全关闭