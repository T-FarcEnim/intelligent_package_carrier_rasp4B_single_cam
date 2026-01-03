#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
obstacle_sensor.py
超声波避障模块（独立逻辑 + 可替换硬件接口）

功能：
1）读取超声波测距
2）判断是否进入避障区
3）执行固定策略：
   优先右转 → 前进 → 左转 → 前进（绕障）
4）输出避障状态给运动控制模块

依赖：
- speed_params.json（读取避障阈值与时序）
"""

import time
import json
from pathlib import Path
from config_loader import ConfigLoader


# =========================
# 硬件接口（可替换为GPIO实际实现）
# =========================
def mock_ultrasonic_read():
    """
    默认提供一个占位读数函数
    实际部署时改成：
    - RPi.GPIO
    - pigpio
    - 或 I2C / UART 距离模块
    """
    return 9999.0  # 默认无障碍


class ObstacleSensor:
    def __init__(self,
                 speed_config="config/speed_params.json",
                 distance_reader=mock_ultrasonic_read):

        loader = ConfigLoader(speed_json=speed_config)
        self.speed_cfg = loader.load_speed_params()

        self.reader = distance_reader

        obst = self.speed_cfg.get("obstacle", {})

        self.safe_distance_cm = float(obst.get("safe_distance_cm", 30.0))
        self.stop_distance_cm = float(obst.get("stop_distance_cm", 12.0))

        # 绕行策略阶段时间（秒）
        self.right_turn_time = float(obst.get("right_turn_time", 0.8))
        self.forward_time_1 = float(obst.get("forward_time_1", 0.6))
        self.left_turn_time = float(obst.get("left_turn_time", 0.8))
        self.forward_time_2 = float(obst.get("forward_time_2", 0.6))

        # 状态机
        self.avoid_mode = False
        self.phase = None
        self.phase_start = 0

    def _start_phase(self, name):
        self.phase = name
        self.phase_start = time.time()

    def _phase_elapsed(self):
        return time.time() - self.phase_start

    def read_distance(self):
        try:
            return float(self.reader())
        except Exception:
            return 9999.0

    def update(self):
        """
        返回避障状态：
        {
            "obstacle_near": bool,
            "need_stop": bool,
            "avoid_mode": bool,
            "phase": str|None,
            "action": "right"|"left"|"forward"|"stop"|None,
            "distance_cm": float
        }
        """

        d = self.read_distance()

        # ---- 靠得太近：强制停止 ----
        if d <= self.stop_distance_cm:
            self.avoid_mode = True
            self._start_phase("STOP_CLOSE")
            return dict(
                obstacle_near=True,
                need_stop=True,
                avoid_mode=True,
                phase="STOP_CLOSE",
                action="stop",
                distance_cm=d
            )

        # ---- 进入避障模式 ----
        if d <= self.safe_distance_cm and not self.avoid_mode:
            self.avoid_mode = True
            self._start_phase("TURN_RIGHT")

        # ---- 离开障碍，退出模式 ----
        if d > self.safe_distance_cm * 1.5 and self.avoid_mode and self.phase is None:
            self.avoid_mode = False

        # ===== 避障状态机 =====
        if not self.avoid_mode:
            return dict(
                obstacle_near=False,
                need_stop=False,
                avoid_mode=False,
                phase=None,
                action=None,
                distance_cm=d
            )

        # --- 各阶段顺序 ---
        if self.phase == "TURN_RIGHT":
            if self._phase_elapsed() < self.right_turn_time:
                return self._state("TURN_RIGHT", "right", d)
            else:
                self._start_phase("FORWARD_1")

        if self.phase == "FORWARD_1":
            if self._phase_elapsed() < self.forward_time_1:
                return self._state("FORWARD_1", "forward", d)
            else:
                self._start_phase("TURN_LEFT")

        if self.phase == "TURN_LEFT":
            if self._phase_elapsed() < self.left_turn_time:
                return self._state("TURN_LEFT", "left", d)
            else:
                self._start_phase("FORWARD_2")

        if self.phase == "FORWARD_2":
            if self._phase_elapsed() < self.forward_time_2:
                return self._state("FORWARD_2", "forward", d)
            else:
                # 绕行结束，恢复主逻辑
                self.phase = None
                return self._state(None, None, d)

        # 默认兜底
        return self._state(self.phase, None, d)

    def _state(self, phase, action, d):
        return dict(
            obstacle_near=(d <= self.safe_distance_cm),
            need_stop=(phase == "STOP_CLOSE"),
            avoid_mode=True,
            phase=phase,
            action=action,
            distance_cm=d
        )


if __name__ == "__main__":
    sensor = ObstacleSensor()
    print("obstacle_sensor 模块测试运行中...")
    for _ in range(10):
        s = sensor.update()
        print(s)
        time.sleep(0.3)