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
- 丢失二维码 → 避障同时朝原方向移动
- 利用摄像头距离和超声波距离差异判断地面障碍
"""

import RPi.GPIO as GPIO
import time
import numpy as np

from camera_module import CameraModule
from qr_detector import QRDetector
from obstacle_sensor import ObstacleSensor
from motion_controller import MotionController

# 电机引脚定义
LEFT_IN1 = 16
LEFT_IN2 = 19
RIGHT_IN3 = 20
RIGHT_IN4 = 21

# PWM频率
PWM_FREQ = 1000  # 1kHz

def init_gpio():
    """初始化GPIO和PWM"""
    global left_pwm1, left_pwm2, right_pwm1, right_pwm2
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 设置电机引脚为输出模式
    for pin in [LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4]:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    
    # 创建PWM对象
    left_pwm1 = GPIO.PWM(LEFT_IN1, PWM_FREQ)
    left_pwm2 = GPIO.PWM(LEFT_IN2, PWM_FREQ)
    right_pwm1 = GPIO.PWM(RIGHT_IN3, PWM_FREQ)
    right_pwm2 = GPIO.PWM(RIGHT_IN4, PWM_FREQ)
    
    # 启动PWM，初始占空比为0
    left_pwm1.start(0)
    left_pwm2.start(0)
    right_pwm1.start(0)
    right_pwm2.start(0)

def drive_motor(left, right):
    """电机驱动函数"""
    if left >= 0:
        left_pwm1.ChangeDutyCycle(min(left*100, 100))
        left_pwm2.ChangeDutyCycle(0)
    else:
        left_pwm1.ChangeDutyCycle(0)
        left_pwm2.ChangeDutyCycle(min(-left*100, 100))
    
    # 右电机
    if right >= 0:
        right_pwm1.ChangeDutyCycle(min(right*100, 100))
        right_pwm2.ChangeDutyCycle(0)
    else:
        right_pwm1.ChangeDutyCycle(0)
        right_pwm2.ChangeDutyCycle(min(-right*100, 100))
    print(f"[DRIVE] L={left:.2f}  R={right:.2f}")


class TrackSystem:
    def __init__(self):
        self.camera = CameraModule()
        self.qr = QRDetector()
        self.obstacle = ObstacleSensor(use_mock=False) 
        self.motion = MotionController()

        # 状态变量
        self.last_valid_qr = None  # 最后一次有效的二维码信息
        self.target_direction = 0  # 目标方向（水平偏移）
        self.qr_lost_timer = 0  # 二维码丢失计时器
        self.estimated_rotation = 0  # 估计的旋转角度
        
        # 障碍物识别状态
        self.obstacle_type = "none"  # none, ground, wall, unknown
        self.ground_obstacle_confirmed = False
        
        # 历史记录
        self.qr_distance_history = []
        self.ultrasonic_history = []
        self.max_history = 10

    def analyze_obstacle_type(self, qr_distance, ultrasonic_distance):
        """
        分析障碍物类型
        1. 如果超声波距离 ≈ 二维码距离 → 可能是墙上的二维码
        2. 如果超声波距离 < 二维码距离 → 地面有障碍
        3. 如果超声波距离 > 二维码距离 → 测量误差或二维码倾斜
        """
        if qr_distance is None or ultrasonic_distance > 300:  # 无效数据
            return "unknown"
        
        ratio = ultrasonic_distance / qr_distance
        
        if 0.8 <= ratio <= 1.2:  # 两者相近
            return "wall_qr"  # 可能是墙上的二维码
        elif ratio < 0.8:  # 超声波测得更近
            return "ground_obstacle"  # 地面有障碍
        else:  # 超声波测得更远
            return "error_or_tilted"  # 测量误差或二维码倾斜

    def run(self):
        self.camera.open()

        try:
            frame_count = 0
            for frame in self.camera.frames():
                frame_count += 1

                # ---- 二维码识别 ----
                qr_state = self.qr.detect(frame)
                
                # 获取超声波距离
                ultrasonic_state = self.obstacle.update()
                ultrasonic_distance = ultrasonic_state["distance_cm"]
                
                # 分析障碍物类型
                if not qr_state.get("lost", True):
                    qr_distance = qr_state["distance_cm"]
                    obstacle_type = self.analyze_obstacle_type(qr_distance, ultrasonic_distance)
                    
                    # 保存历史数据用于滤波
                    self.qr_distance_history.append(qr_distance)
                    self.ultrasonic_history.append(ultrasonic_distance)
                    if len(self.qr_distance_history) > self.max_history:
                        self.qr_distance_history.pop(0)
                    if len(self.ultrasonic_history) > self.max_history:
                        self.ultrasonic_history.pop(0)
                        
                    # 使用历史中值滤波
                    if len(self.qr_distance_history) >= 3:
                        qr_median = np.median(self.qr_distance_history)
                        us_median = np.median(self.ultrasonic_history)
                        obstacle_type = self.analyze_obstacle_type(qr_median, us_median)
                    
                    self.obstacle_type = obstacle_type
                    
                    # 更新目标方向
                    self.target_direction = qr_state["offsets"]["cx_off"]
                    
                    # 如果是地面障碍，标记确认
                    if obstacle_type == "ground_obstacle" and ultrasonic_distance < 30:
                        self.ground_obstacle_confirmed = True
                    else:
                        self.ground_obstacle_confirmed = False
                        
                    # 重置丢失计时器
                    self.qr_lost_timer = 0
                    
                    # 打印状态（调试用）
                    if frame_count % 10 == 0:
                        print(f"帧 {frame_count}: 二维码距离={qr_distance:.1f}cm, "
                              f"超声距离={ultrasonic_distance:.1f}cm, "
                              f"障碍类型={obstacle_type}")
                        if self.ground_obstacle_confirmed:
                            print(f"  确认地面障碍！")
                
                else:
                    # 丢失二维码
                    self.qr_lost_timer += 1
                    
                    # 如果之前有有效二维码，使用历史数据
                    if self.last_valid_qr is not None:
                        qr_state = self.last_valid_qr.copy()
                        qr_state["lost"] = True
                        
                        # 更新状态
                        qr_state["obstacle_type"] = self.obstacle_type
                        qr_state["ultrasonic_distance"] = ultrasonic_distance
                        qr_state["qr_lost_timer"] = self.qr_lost_timer
                        qr_state["target_direction"] = self.target_direction
                    else:
                        qr_state["lost"] = True
                        qr_state["obstacle_type"] = "unknown"
                        qr_state["ultrasonic_distance"] = ultrasonic_distance
                        qr_state["qr_lost_timer"] = self.qr_lost_timer
                        qr_state["target_direction"] = 0

                # ---- 更新障碍物状态 ----
                # 添加额外信息到obstacle_state
                ultrasonic_state["obstacle_type"] = self.obstacle_type
                ultrasonic_state["ground_obstacle_confirmed"] = self.ground_obstacle_confirmed
                ultrasonic_state["qr_lost"] = qr_state.get("lost", True)
                
                # 如果二维码丢失但有地面障碍确认，强制进入避障模式
                if (qr_state.get("lost", True) and 
                    self.ground_obstacle_confirmed and 
                    ultrasonic_distance < 30):
                    ultrasonic_state["obstacle_near"] = True
                    ultrasonic_state["need_stop"] = True
                
                # 估计旋转角度（基于上一次的运动）
                if frame_count > 1:
                    # 获取上一次的运动命令并估计旋转
                    pass  # 这部分在motion_controller中实现更准确

                # ---- 运动计算 ----
                motion = self.motion.compute(
                    qr_state, 
                    ultrasonic_state,
                    extra_info={
                        "frame_count": frame_count,
                        "target_direction": self.target_direction,
                        "qr_lost_timer": self.qr_lost_timer,
                        "estimated_rotation": self.estimated_rotation,
                        "obstacle_type": self.obstacle_type
                    }
                )
                
                # 更新估计的旋转角度
                if "rotation_estimate" in motion:
                    self.estimated_rotation = motion["rotation_estimate"]

                # ---- 电机输出 ----
                drive_motor(motion["left_speed"], motion["right_speed"])

                # 调试输出
                if frame_count % 10 == 0:
                    print(f"运动: 模式={motion.get('mode', '未知')}, "
                        f"左轮={motion.get('left_speed', 0):.3f}, "
                        f"右轮={motion.get('right_speed', 0):.3f}")
                    
                    if qr_state.get("lost", True):
                        print(f"  已丢失二维码 {self.qr_lost_timer}帧, "
                              f"目标方向={self.target_direction:.1f}, "
                              f"估计旋转={self.estimated_rotation:.1f}度")

                # 保存最后一次有效二维码信息
                if not qr_state.get("lost", True):
                    self.last_valid_qr = qr_state.copy()

                # 控制循环频率
                time.sleep(0.02)

        finally:
            self.camera.close()

if __name__ == "__main__":
    init_gpio()
    system = TrackSystem()
    print("自动循迹系统启动...")
    system.run()