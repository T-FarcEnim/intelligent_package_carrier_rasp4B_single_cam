#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
motion_controller.py
运动控制模块（实时向量演算 + 模拟 PID）

功能：
1）根据二维码识别结果与避障状态决定运动
2）实现向量公式：
   单侧轮 = 停车*转弯*(直行补偿*直行基础)
          + 停车*直行*(转弯基础 + 方向参数*方向补偿)
3）二维码优先控制方向，距离 + 上下趋势控制直行速度
4）避障优先级高于跟踪
5）记录上下坡趋势（最近一次趋势记忆 + 动态补偿）

输入：
- qr_state = {
    "lost": bool,
    "distance_cm": float,
    "offsets": {
        "cx_off","cy_off",
        "yaw_trend","pitch_trend"
    }
  }
- obstacle_state = obstacle_sensor.update()

输出：
{
    "left_speed": float,
    "right_speed": float,
    "mode": str
}
"""

from config_loader import ConfigLoader
import math
import time


class MotionController:
    def __init__(self,
                 speed_config="config/speed_params.json"):

        loader = ConfigLoader(speed_json=speed_config)
        self.cfg = loader.load_speed_params()

        sp = self.cfg

        self.base_forward = float(sp.get("base_forward", 0.35))
        self.forward_comp = float(sp.get("forward_comp", 0.8))

        self.base_turn = float(sp.get("base_turn", 0.28))
        self.turn_comp = float(sp.get("turn_comp", 0.45))

        self.dead_zone_px = float(sp.get("dead_zone_px", 25.0))

        pid = sp.get("pid", {})
        self.kp_dist = float(pid.get("kp_dist", 0.015))
        self.kp_pitch = float(pid.get("kp_pitch", 0.01))
        self.kp_yaw = float(pid.get("kp_yaw", 0.004))

        # 坡道状态记忆
        self.last_slope = "flat"
        self.on_slope = False
        self.last_mode = "idle"

        # 安全限制
        self.max_speed = float(sp.get("max_speed", 1.0))
        self.min_speed = float(sp.get("min_speed", -1.0))

        # 跟踪距离阈值
        self.track_far = float(sp.get("track_far_cm", 120.0))
        self.stop_near = float(sp.get("stop_near_cm", 22.0))

    # =======================
    # 工具函数
    # =======================
    def _clamp(self, v):
        return max(self.min_speed, min(self.max_speed, v))

    def _vector_formula(self, stop_flag, turn_flag,
                        forward_base, turn_base,
                        turn_dir):
        """
        对应题中运动向量公式
        """
        return (
            stop_flag * turn_flag * (self.forward_comp * forward_base)
            + stop_flag * (1 - turn_flag)
            * (turn_base + turn_dir * self.turn_comp * turn_base)
        )

    # =======================
    # 避障优先输出
    # =======================
    def _avoid_motion(self, obst):
        action = obst["action"]

        if action == "stop":
            return 0.0, 0.0, "avoid_stop"

        if action == "right":
            return -self.base_turn, self.base_turn, "avoid_right"

        if action == "left":
            return self.base_turn, -self.base_turn, "avoid_left"

        if action == "forward":
            v = self.base_forward * 0.6
            return v, v, "avoid_forward"

        return 0.0, 0.0, "avoid_idle"

    # =======================
    # 上下坡趋势修正
    # pitch_trend > 0 说明二维码偏下（上坡）
    # =======================
    def _slope_adjust(self, pitch_trend, dist):
        if abs(pitch_trend) < 2:
            self.on_slope = False
            self.last_slope = "flat"
            return 0.0

        if pitch_trend > 0:
            self.last_slope = "up"
            self.on_slope = True
            return +0.05

        else:
            self.last_slope = "down"
            self.on_slope = True
            return -0.05

    # =======================
    # 跟踪控制核心
    # =======================
    def compute(self, qr_state, obstacle_state):
        """
        统一外部调用接口
        """

        # ---------- 避障优先 ----------
        if obstacle_state["avoid_mode"]:
            l, r, mode = self._avoid_motion(obstacle_state)
            self.last_mode = mode
            return dict(left_speed=l, right_speed=r, mode=mode)

        # ---------- 丢失二维码 ----------
        if qr_state.get("lost", True):
            # 保持原状态（惰性维持）
            self.last_mode = "lost_keep"
            return dict(
                left_speed=0.0,
                right_speed=0.0,
                mode="lost_wait"
            )

        # ========== 二维码存在 ==========
        dist = qr_state["distance_cm"]
        off = qr_state["offsets"]

        cx = off["cx_off"]
        pitch = off["pitch_trend"]
        yaw = off["yaw_trend"]

        # ---------- 距离相关 ----------
        if dist <= self.stop_near:
            self.last_mode = "target_near_stop"
            return dict(left_speed=0, right_speed=0, mode="near_stop")

        # ---------- 上坡/下坡补偿 ----------
        slope_adj = self._slope_adjust(pitch, dist)

        # ---------- 前进速度 PID ----------
        forward = (
            self.base_forward
            + self.kp_dist * (dist - self.stop_near)
            + self.kp_pitch * slope_adj
        )
        forward = self._clamp(forward)

        # ---------- 转向判定（中央死区） ----------
        if abs(cx) < self.dead_zone_px:
            turn_dir = 0.0
            turn_flag = 0
        else:
            turn_dir = -math.copysign(1.0, cx)
            turn_flag = 1

        turn = self.kp_yaw * yaw
        turn = self._clamp(turn)

        stop_flag = 1  # 允许运行

        # ---------- 向量公式 ----------
        left = self._vector_formula(
            stop_flag,
            turn_flag,
            forward,
            self.base_turn,
            turn_dir
        )

        right = self._vector_formula(
            stop_flag,
            1 - turn_flag,
            forward,
            self.base_turn,
            -turn_dir
        )

        left = self._clamp(left)
        right = self._clamp(right)

        self.last_mode = "track"
        return dict(
            left_speed=left,
            right_speed=right,
            mode="track_follow"
        )


if __name__ == "__main__":
    print("motion_controller 模块加载成功")