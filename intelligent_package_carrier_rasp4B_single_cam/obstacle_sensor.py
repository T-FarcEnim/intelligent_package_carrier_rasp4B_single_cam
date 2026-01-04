#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
obstacle_sensor.py
超声波避障模块（独立逻辑 + GPIO硬件接口）

功能：
1）读取超声波测距（HC-SR04模块）
2）判断是否进入避障区
3）执行固定策略：
   优先右转 → 前进 → 左转 → 前进（绕障）
4）输出避障状态给运动控制模块

依赖：
- speed_params.json（读取避障阈值与时序）
- RPi.GPIO
"""

import RPi.GPIO as GPIO
import time
import json
from pathlib import Path
from config_loader import ConfigLoader


# =========================
# GPIO引脚定义（BCM编号）
# =========================
ULTRASONIC_TRIG = 27  # 触发引脚
ULTRASONIC_ECHO = 22  # 回波引脚

# =========================
# 超声波硬件接口
# =========================
def init_ultrasonic():
    """初始化超声波传感器GPIO"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ULTRASONIC_TRIG, GPIO.OUT)
    GPIO.setup(ULTRASONIC_ECHO, GPIO.IN)
    GPIO.output(ULTRASONIC_TRIG, GPIO.LOW)
    time.sleep(0.5)  # 让传感器稳定
    print("[Ultrasonic] GPIO初始化完成")

def read_ultrasonic_distance():
    """
    使用HC-SR04模块读取距离（厘米）
    返回：距离（cm），出错时返回9999.0
    """
    try:
        # 确保Trig为低电平
        GPIO.output(ULTRASONIC_TRIG, GPIO.LOW)
        time.sleep(0.0002)  # 2μs
        
        # 发送10μs的高电平脉冲
        GPIO.output(ULTRASONIC_TRIG, GPIO.HIGH)
        time.sleep(0.00001)  # 10μs
        GPIO.output(ULTRASONIC_TRIG, GPIO.LOW)
        
        # 等待回波开始（Echo变高）
        timeout_start = time.time()
        while GPIO.input(ULTRASONIC_ECHO) == GPIO.LOW:
            if time.time() - timeout_start > 0.1:  # 100ms超时
                return 9999.0
        
        pulse_start = time.time()
        
        # 等待回波结束（Echo变低）
        timeout_start = time.time()
        while GPIO.input(ULTRASONIC_ECHO) == GPIO.HIGH:
            if time.time() - timeout_start > 0.1:  # 100ms超时
                return 9999.0
        
        pulse_end = time.time()
        
        # 计算距离（声速343m/s = 17150 cm/s）
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        
        # 有效距离范围限制（2cm - 400cm）
        if distance < 2.0:
            return 2.0
        elif distance > 400.0:
            return 400.0
        else:
            return round(distance, 1)  # 保留1位小数
            
    except Exception as e:
        print(f"[Ultrasonic] 读取错误: {e}")
        return 9999.0

def mock_ultrasonic_read():
    """
    模拟超声波读数（用于测试）
    实际部署时使用 read_ultrasonic_distance()
    """
    return 100.0  # 默认无障碍


class ObstacleSensor:
    def __init__(self,
                 speed_config="config/speed_params.json",
                 use_mock=False):
        """
        初始化避障传感器
        
        参数：
        - speed_config: 配置文件路径
        - use_mock: 是否使用模拟读数（测试用）
        """
        
        # 加载配置文件
        loader = ConfigLoader(speed_json=speed_config)
        self.speed_cfg = loader.load_speed_params()
        
        # 设置读数函数
        if use_mock:
            init_ultrasonic()
            self.reader = mock_ultrasonic_read
            print("[ObstacleSensor] 使用模拟读数模式")
        else:
            # 初始化GPIO并使用真实读数
            init_ultrasonic()
            self.reader = read_ultrasonic_distance
            print("[ObstacleSensor] 使用真实超声波传感器")
        
        # 从配置读取参数
        obst = self.speed_cfg.get("obstacle", {})
        
        # 距离阈值（厘米）
        self.safe_distance_cm = float(obst.get("safe_distance_cm", 30.0))
        self.stop_distance_cm = float(obst.get("stop_distance_cm", 12.0))
        
        # 绕行策略阶段时间（秒） - 注意配置单位应为秒
        # 修复：配置中的时间单位是秒，但值可能过大，需要检查
        right_turn_time = float(obst.get("right_turn_time", 0.8))
        forward_time_1 = float(obst.get("forward_time_1", 0.6))
        left_turn_time = float(obst.get("left_turn_time", 0.8))
        forward_time_2 = float(obst.get("forward_time_2", 0.6))
        
        # 检查时间参数是否合理（防止配置错误导致过大值）
        max_reasonable_time = 5.0  # 最大合理时间5秒
        
        self.right_turn_time = min(right_turn_time, max_reasonable_time) 
        self.forward_time_1 = min(forward_time_1, max_reasonable_time) 
        self.left_turn_time = min(left_turn_time, max_reasonable_time) 
        self.forward_time_2 = min(forward_time_2, max_reasonable_time) 
        
        print(f"[ObstacleSensor] 避障参数: 安全距离={self.safe_distance_cm}cm, "
              f"停止距离={self.stop_distance_cm}cm")
        print(f"[ObstacleSensor] 绕行时间: 右转={self.right_turn_time:.2f}s, "
              f"前进1={self.forward_time_1:.2f}s, 左转={self.left_turn_time:.2f}s, "
              f"前进2={self.forward_time_2:.2f}s")
        
        # 状态机
        self.avoid_mode = False
        self.phase = None
        self.phase_start = 0
        self.last_distance = 9999.0
        self.consecutive_readings = []  # 用于滤波的连续读数
        self.filter_size = 5  # 中值滤波窗口大小
        
        # 安全距离补偿（根据历史读数动态调整）
        self.dynamic_safe_distance = self.safe_distance_cm
        
    def _start_phase(self, name):
        """开始一个新阶段"""
        self.phase = name
        self.phase_start = time.time()
        print(f"[ObstacleSensor] 进入阶段: {name}")
    
    def _phase_elapsed(self):
        """获取当前阶段已过时间"""
        return time.time() - self.phase_start
    
    def _median_filter(self, new_distance):
        """中值滤波，减少噪声干扰"""
        self.consecutive_readings.append(new_distance)
        if len(self.consecutive_readings) > self.filter_size:
            self.consecutive_readings.pop(0)
        
        # 计算中值
        sorted_readings = sorted(self.consecutive_readings)
        median_index = len(sorted_readings) // 2
        return sorted_readings[median_index]
    
    def _dynamic_threshold_adjust(self, distance):
        """动态调整安全距离阈值"""
        # 如果连续几次读数都很稳定，可以适当放宽安全距离
        if len(self.consecutive_readings) >= self.filter_size:
            readings_range = max(self.consecutive_readings) - min(self.consecutive_readings)
            
            if readings_range < 5.0:  # 读数稳定
                # 稍微放宽安全距离，避免频繁触发
                self.dynamic_safe_distance = self.safe_distance_cm * 1.1
            else:
                # 读数波动大，使用保守值
                self.dynamic_safe_distance = self.safe_distance_cm
        
        return self.dynamic_safe_distance
    
    def read_distance(self):
        """读取并滤波处理距离数据"""
        try:
            raw_distance = float(self.reader())
            
            # 无效读数处理
            if raw_distance <= 0:
                print(f"[ObstacleSensor] 无效读数: {raw_distance}")
                return self.last_distance  # 返回上次有效读数
            
            if raw_distance >= 9999.0:
                print(f"[ObstacleSensor] 无效读数: {raw_distance}")
                return 9999 
            
            # 中值滤波
            filtered_distance = self._median_filter(raw_distance)
            self.last_distance = filtered_distance
            
            # 动态调整阈值
            self._dynamic_threshold_adjust(filtered_distance)
            
            return filtered_distance
            
        except Exception as e:
            print(f"[ObstacleSensor] 读取错误: {e}")
            return self.last_distance
    
    def update(self):
        """
        更新避障状态
        
        返回避障状态字典：
        {
            "obstacle_near": bool,        # 是否有障碍物在安全距离内
            "need_stop": bool,            # 是否需要紧急停止
            "avoid_mode": bool,           # 是否处于避障模式
            "phase": str|None,            # 当前避障阶段
            "action": "right"|"left"|"forward"|"stop"|None,  # 建议动作
            "distance_cm": float,         # 当前距离（滤波后）
            "safe_distance": float        # 当前使用的安全距离阈值
        }
        """
        
        # 读取当前距离
        d = self.read_distance()
        
        # 动态安全距离
        current_safe_distance = self.dynamic_safe_distance
        
        # ---- 靠得太近：强制停止 ----
        if d <= self.stop_distance_cm:
            self.avoid_mode = True
            self._start_phase("STOP_CLOSE")
            return self._create_state(
                obstacle_near=True,
                need_stop=True,
                phase="STOP_CLOSE",
                action="stop",
                distance=d,
                safe_distance=current_safe_distance
            )
        
        # ---- 进入避障模式 ----
        if d <= current_safe_distance and not self.avoid_mode:
            self.avoid_mode = True
            self._start_phase("TURN_RIGHT")
            print(f"[ObstacleSensor] 检测到障碍物！距离: {d:.1f}cm < 安全距离: {current_safe_distance:.1f}cm")
        
        # ---- 离开障碍，退出模式 ----
        if (d > current_safe_distance * 1.8 and 
            self.avoid_mode and 
            self.phase is None):
            self.avoid_mode = False
            print("[ObstacleSensor] 障碍物已避开，退出避障模式")
        
        # ===== 避障状态机 =====
        if not self.avoid_mode:
            return self._create_state(
                obstacle_near=False,
                need_stop=False,
                phase=None,
                action=None,
                distance=d,
                safe_distance=current_safe_distance
            )
        
        # --- 各阶段顺序执行 ---
        
        # 阶段1: 右转
        if self.phase == "TURN_RIGHT":
            if self._phase_elapsed() < self.right_turn_time:
                return self._create_state(
                    obstacle_near=True,
                    need_stop=False,
                    phase="TURN_RIGHT",
                    action="right",
                    distance=d,
                    safe_distance=current_safe_distance
                )
            else:
                print(f"[ObstacleSensor] 右转完成，用时{self._phase_elapsed():.2f}s")
                self._start_phase("FORWARD_1")
        
        # 阶段2: 前进1
        if self.phase == "FORWARD_1":
            if self._phase_elapsed() < self.forward_time_1:
                return self._create_state(
                    obstacle_near=True,
                    need_stop=False,
                    phase="FORWARD_1",
                    action="forward",
                    distance=d,
                    safe_distance=current_safe_distance
                )
            else:
                print(f"[ObstacleSensor] 前进1完成，用时{self._phase_elapsed():.2f}s")
                self._start_phase("TURN_LEFT")
        
        # 阶段3: 左转
        if self.phase == "TURN_LEFT":
            if self._phase_elapsed() < self.left_turn_time:
                return self._create_state(
                    obstacle_near=True,
                    need_stop=False,
                    phase="TURN_LEFT",
                    action="left",
                    distance=d,
                    safe_distance=current_safe_distance
                )
            else:
                print(f"[ObstacleSensor] 左转完成，用时{self._phase_elapsed():.2f}s")
                self._start_phase("FORWARD_2")
        
        # 阶段4: 前进2
        if self.phase == "FORWARD_2":
            if self._phase_elapsed() < self.forward_time_2:
                return self._create_state(
                    obstacle_near=True,
                    need_stop=False,
                    phase="FORWARD_2",
                    action="forward",
                    distance=d,
                    safe_distance=current_safe_distance
                )
            else:
                # 绕行结束，恢复主逻辑
                print(f"[ObstacleSensor] 前进2完成，用时{self._phase_elapsed():.2f}s，绕行结束")
                self.phase = None
                # 等待主逻辑重新评估是否继续避障
                time.sleep(0.1)  # 短暂暂停
                return self._create_state(
                    obstacle_near=(d <= current_safe_distance),
                    need_stop=False,
                    phase=None,
                    action=None,
                    distance=d,
                    safe_distance=current_safe_distance
                )
        
        # 默认状态（阶段之间或未知阶段）
        return self._create_state(
            obstacle_near=(d <= current_safe_distance),
            need_stop=False,
            phase=self.phase,
            action=None,
            distance=d,
            safe_distance=current_safe_distance
        )
    
    def _create_state(self, obstacle_near, need_stop, phase, action, distance, safe_distance):
        """创建状态字典的统一方法"""
        return {
            "obstacle_near": obstacle_near,
            "need_stop": need_stop,
            "avoid_mode": self.avoid_mode,
            "phase": phase,
            "action": action,
            "distance_cm": distance,
            "safe_distance": safe_distance
        }
    
    def get_status_string(self):
        """获取状态字符串（用于显示）"""
        d = self.last_distance
        status = f"距离: {d:.1f}cm"
        
        if self.avoid_mode:
            status += f" | 避障模式: {self.phase if self.phase else '无'}"
            if self.phase:
                elapsed = self._phase_elapsed()
                if self.phase == "TURN_RIGHT":
                    total = self.right_turn_time
                elif self.phase == "FORWARD_1":
                    total = self.forward_time_1
                elif self.phase == "TURN_LEFT":
                    total = self.left_turn_time
                elif self.phase == "FORWARD_2":
                    total = self.forward_time_2
                else:
                    total = 0
                
                if total > 0:
                    status += f" ({elapsed:.1f}/{total:.1f}s)"
        
        if d <= self.stop_distance_cm:
            status += " | 紧急停止距离！"
        elif d <= self.dynamic_safe_distance:
            status += f" | 障碍物接近 (安全距离: {self.dynamic_safe_distance:.1f}cm)"
        
        return status
    
    def cleanup(self):
        """清理资源"""
        print("[ObstacleSensor] 清理资源")
        # 注意：GPIO.cleanup()应在主程序中统一调用


if __name__ == "__main__":
    print("=== 超声波避障模块测试 ===")
    print("测试选项:")
    print("1. 使用模拟模式（无硬件）")
    print("2. 使用真实超声波传感器")
    
    choice = input("请选择 (1 或 2): ").strip()
    
    if choice == "1":
        sensor = ObstacleSensor(use_mock=True)
        test_mode = "模拟"
    else:
        sensor = ObstacleSensor(use_mock=False)
        test_mode = "真实"
    
    print(f"\n使用{test_mode}传感器进行测试")
    print("按 Ctrl+C 停止测试\n")
    
    try:
        while True:
            state = sensor.update()
            print(f"状态: {sensor.get_status_string()}")
            
            if state["action"]:
                print(f"  建议动作: {state['action']}")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n测试结束")
    finally:
        if choice != "1":
            GPIO.cleanup()
        print("资源已清理")
