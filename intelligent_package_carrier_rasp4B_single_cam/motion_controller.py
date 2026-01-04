#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
motion_controller.py
运动控制模块（实时向量演算 + 智能避障）

功能：
1）根据二维码识别结果与避障状态决定运动
2）当二维码存在时，比较摄像头距离和超声波距离判断障碍类型
3）当二维码丢失时，智能避障同时朝原方向移动
4）记录旋转角度，避免过度偏离目标方向
"""

from config_loader import ConfigLoader
import math
import time
import numpy as np


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

        # 安全限制
        self.max_speed = float(sp.get("max_speed", 1.0))
        self.min_speed = float(sp.get("min_speed", -1.0))

        # 跟踪距离阈值
        self.track_far = float(sp.get("track_far_cm", 120.0))
        self.stop_near = float(sp.get("stop_near_cm", 22.0))
        
        # 避障参数
        self.obstacle_safe_distance = float(sp.get("obstacle_safe_distance", 30.0))
        self.obstacle_stop_distance = float(sp.get("obstacle_stop_distance", 12.0))
        
        # 智能搜索参数
        self.search_base_speed = float(sp.get("search_base_speed", 0.2))
        self.search_turn_gain = float(sp.get("search_turn_gain", 0.3))
        self.max_rotation_deviation = float(sp.get("max_rotation_deviation", 60.0))  # 最大允许偏离角度
        
        # 状态变量
        self.last_mode = "idle"
        self.cumulative_rotation = 0  # 累积旋转角度
        self.last_left_speed = 0
        self.last_right_speed = 0
        self.last_update_time = time.time()
        
        # 方向记忆
        self.target_direction_history = []
        self.max_history = 20
        self.Halt = 1

    # =======================
    # 工具函数
    # =======================
    def _clamp(self, v):
        return max(self.min_speed, min(self.max_speed, v))
    
    def _estimate_rotation(self, left_speed, right_speed, dt):
        """
        估计旋转角度（度/秒）
        简化模型：角度变化 ∝ (右速度 - 左速度)
        """
        if dt <= 0:
            return 0
            
        # 基本参数：最大速度差对应的最大旋转率
        max_speed_diff = 0.5  # 假设最大速度差
        max_rotation_rate = 180.0  # 度/秒
        
        # 计算速度差
        speed_diff = right_speed - left_speed
        
        # 归一化并计算旋转率
        rotation_rate = (speed_diff / max_speed_diff) * max_rotation_rate
        
        # 积分得到角度变化
        rotation_change = rotation_rate * dt
        
        return rotation_change
    
    def _update_rotation_estimate(self, left_speed, right_speed):
        """更新累积旋转角度估计"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        if dt > 0:
            rotation_change = self._estimate_rotation(
                self.last_left_speed, 
                self.last_right_speed, 
                dt
            )
            self.cumulative_rotation += rotation_change
            
            # 限制在合理范围内
            if abs(self.cumulative_rotation) > 360:
                self.cumulative_rotation = self.cumulative_rotation % 360
        
        self.last_left_speed = left_speed
        self.last_right_speed = right_speed
        self.last_update_time = current_time
        
        return self.cumulative_rotation

    # =======================
    # 智能避障与方向保持
    # =======================
    def _smart_avoidance(self, obstacle_state, target_direction, cumulative_rotation):
        """
        智能避障：在避障的同时尽量保持/回归目标方向
        
        关键策略：
        1. 如果障碍物很近需要紧急避让，优先避障
        2. 如果障碍物中等距离，避障+方向修正
        3. 如果障碍物较远但二维码丢失，朝目标方向搜索前进
        """
        
        action = obstacle_state.get("action", None)
        distance = obstacle_state.get("distance_cm", 9999.0)
        obstacle_type = obstacle_state.get("obstacle_type", "unknown")
        ground_obstacle = obstacle_state.get("ground_obstacle_confirmed", False)
        if(obstacle_state.get("need_stop") is True):
            self.Halt = 0
        else:
            self.Halt = 1
        
        # 紧急避障：距离非常近
        if distance <= self.obstacle_stop_distance:
            # 紧急停止并轻微转向避开
            if target_direction > 0:  # 目标在右边，向左转避开
                left = -self.base_turn * 0.3 * self.Halt
                right = self.base_turn * 0.3 * self.Halt
                mode = "emergency_avoid_left"
            else:  # 目标在左边，向右转避开
                left = self.base_turn * 0.3 * self.Halt
                right = -self.base_turn * 0.3 * self.Halt
                mode = "emergency_avoid_right"
            
            # 限制速度
            left = self._clamp(left)
            right = self._clamp(right)
            
            return left, right, mode
        
        # 标准避障动作
        if action == "stop":
            # 停止时根据目标方向轻微调整朝向
            if abs(cumulative_rotation) < self.max_rotation_deviation:
                # 尝试转向目标方向
                if target_direction > 10:
                    left = self.base_turn * 0.15* self.Halt
                    right = -self.base_turn * 0.15* self.Halt
                    mode = "stop_adjust_right"
                elif target_direction < -10:
                    left = -self.base_turn * 0.15* self.Halt
                    right = self.base_turn * 0.15* self.Halt
                    mode = "stop_adjust_left"
                else:
                    left = 0.0
                    right = 0.0
                    mode = "stop"
            else:
                # 已经偏离太多，停止等待
                left = 0.0
                right = 0.0
                mode = "stop_too_far"
                
            return left, right, mode
        
        elif action == "right":
            # 右转避障，但根据目标方向调整
            if target_direction < -20:  # 目标在左边，减小右转幅度
                factor = 0.6
                left = -self.base_turn * factor* self.Halt
                right = self.base_turn * factor* self.Halt
                mode = "avoid_right_soft"
            else:
                left = -self.base_turn* self.Halt
                right = self.base_turn* self.Halt
                mode = "avoid_right"
            
            # 如果是地面障碍，添加轻微前进分量
            if ground_obstacle:
                forward_component = self.search_base_speed * 0.2
                left = self._clamp(left + forward_component)
                right = self._clamp(right + forward_component)
                
            return left, right, mode
        
        elif action == "left":
            # 左转避障，但根据目标方向调整
            if target_direction > 20:  # 目标在右边，减小左转幅度
                factor = 0.6
                left = self.base_turn * factor* self.Halt
                right = -self.base_turn * factor* self.Halt
                mode = "avoid_left_soft"
            else:
                left = self.base_turn* self.Halt
                right = -self.base_turn* self.Halt
                mode = "avoid_left"
            
            # 如果是地面障碍，添加轻微前进分量
            if ground_obstacle:
                forward_component = self.search_base_speed * 0.2
                left = self._clamp(left + forward_component)
                right = self._clamp(right + forward_component)
                
            return left, right, mode
        
        elif action == "forward":
            # 前进避障，根据目标方向差速
            base_speed = self.base_forward * 0.6
            
            if target_direction > 20:  # 目标在右边
                left = base_speed * 1.2* self.Halt
                right = base_speed * 0.8* self.Halt
                mode = "forward_lean_right"
            elif target_direction < -20:  # 目标在左边
                left = base_speed * 0.8* self.Halt
                right = base_speed * 1.2* self.Halt
                mode = "forward_lean_left"
            else:
                left = base_speed* self.Halt
                right = base_speed* self.Halt
                mode = "forward"
            
            return left, right, mode
        
        else:
            # 没有明确避障动作，朝目标方向搜索前进
            return self._search_toward_target(target_direction, cumulative_rotation)

    def _search_toward_target(self, target_direction, cumulative_rotation):
        """
        朝目标方向搜索前进
        策略：根据目标方向偏差和已旋转角度调整运动
        """
        
        # 限制目标方向在合理范围内
        target_direction_clipped = max(-100, min(100, target_direction))
        
        # 如果已经偏离太多，先纠正方向
        if abs(cumulative_rotation) > self.max_rotation_deviation:
            # 需要反向旋转纠正
            if cumulative_rotation > 0:  # 已经右转太多，需要左转
                left = -self.search_base_speed * 0.8* self.Halt
                right = self.search_base_speed * 0.8* self.Halt
                mode = "search_correct_left"
            else:  # 已经左转太多，需要右转
                left = self.search_base_speed * 0.8* self.Halt
                right = -self.search_base_speed * 0.8* self.Halt
                mode = "search_correct_right"
        else:
            # 根据目标方向调整
            direction_factor = target_direction_clipped / 100.0  # -1到1
            
            if abs(direction_factor) < 0.1:  # 目标在正前方
                left = self.search_base_speed* self.Halt
                right = self.search_base_speed* self.Halt
                mode = "search_forward"
            else:
                # 差速转向
                turn_strength = abs(direction_factor) * self.search_turn_gain
                
                if direction_factor > 0:  # 目标在右边
                    left = self.search_base_speed* self.Halt
                    right = self.search_base_speed * (1.0 - turn_strength)* self.Halt
                    mode = "search_right"
                else:  # 目标在左边
                    left = self.search_base_speed * (1.0 - turn_strength)* self.Halt
                    right = self.search_base_speed* self.Halt
                    mode = "search_left"
        
        # 确保速度不为负
        left = max(0, left)
        right = max(0, right)
        
        return left, right, mode

    # =======================
    # 正常跟踪模式
    # =======================
    def _normal_tracking(self, qr_state, obstacle_state):
        """正常二维码跟踪模式"""
        dist = qr_state["distance_cm"]
        off = qr_state["offsets"]
        
        cx = off["cx_off"]
        pitch = off["pitch_trend"]
        yaw = off["yaw_trend"]
        
        # 检查是否需要避障
        ultrasonic_distance = obstacle_state.get("distance_cm", 9999.0)
        obstacle_type = obstacle_state.get("obstacle_type", "unknown")
        
        # 如果确认有地面障碍且距离近，优先避障
        if (obstacle_type == "ground_obstacle" and 
            ultrasonic_distance < self.obstacle_safe_distance and
            ultrasonic_distance < dist * 0.8):  # 超声波比二维码距离近很多
            
            print(f"[跟踪] 检测到地面障碍！二维码距离={dist:.1f}cm, 超声距离={ultrasonic_distance:.1f}cm")
            
            # 轻度避障，保持跟踪
            if cx > 0:  # 二维码在右边，向左轻微避让
                left = self.base_forward * 0.7
                right = self.base_forward * 0.9
                mode = "track_avoid_left"
            else:  # 二维码在左边，向右轻微避让
                left = self.base_forward * 0.9
                right = self.base_forward * 0.7
                mode = "track_avoid_right"
            
            # 俯仰补偿
            if pitch > 5:  # 上坡
                left *= 1.1
                right *= 1.1
            elif pitch < -5:  # 下坡
                left *= 0.9
                right *= 0.9
                
            return left, right, mode
        
        # 正常跟踪逻辑
        print(f"[跟踪] 距离={dist:.1f}cm, 水平偏移={cx:.1f}px, 俯仰={pitch:.1f}")

        # 距离相关
        if dist <= self.stop_near:
            self.last_mode = "target_near_stop"
            print(f"[跟踪] 距离太近 ({dist:.1f}cm <= {self.stop_near}cm)，停止")
            return 0.0, 0.0, "near_stop"

        # 前进速度计算
        distance_factor = min(1.0, (dist - self.stop_near) / (self.track_far - self.stop_near))
        
        forward = (
            self.base_forward
            + distance_factor * (self.max_speed - self.base_forward)
        )
        
        # 俯仰补偿（上下坡）
        if pitch > 5:  # 上坡
            forward = forward * 1.1
            print(f"[跟踪] 上坡检测，增加速度")
        elif pitch < -5:  # 下坡
            forward = forward * 0.9
            print(f"[跟踪] 下坡检测，减少速度")
        
        forward = min(forward, self.max_speed)
        
        # 转向控制
        if abs(cx) < self.dead_zone_px:
            # 在死区内，直行
            left_speed = forward
            right_speed = forward
            mode = "forward"
            print(f"[跟踪] 直行，速度={forward:.3f}")
        else:
            # 需要转向
            turn_strength = min(abs(cx) / 100.0, 1.0)
            
            if cx > 0:  # 二维码在右边，需要右转
                left_speed = forward
                right_speed = forward * (1.0 - turn_strength * self.turn_comp)
                mode = "turn_right"
                print(f"[跟踪] 右转，左轮={left_speed:.3f}, 右轮={right_speed:.3f}")
            else:  # 二维码在左边，需要左转
                left_speed = forward * (1.0 - turn_strength * self.turn_comp)
                right_speed = forward
                mode = "turn_left"
                print(f"[跟踪] 左转，左轮={left_speed:.3f}, 右轮={right_speed:.3f}")
        
        # 确保速度不为负
        left_speed = max(0, left_speed)
        right_speed = max(0, right_speed)
        
        self.last_mode = mode
        return left_speed, right_speed, mode

    # =======================
    # 主计算函数
    # =======================
    def compute(self, qr_state, obstacle_state, extra_info=None):
        """
        主计算函数
        优先跟踪，但在障碍物存在时智能避障并保持方向
        """
        if extra_info is None:
            extra_info = {}
        
        target_direction = extra_info.get("target_direction", 0)
        qr_lost_timer = extra_info.get("qr_lost_timer", 0)
        
        # 更新旋转角度估计
        cumulative_rotation = self._update_rotation_estimate(
            self.last_left_speed, 
            self.last_right_speed
        )
        
        # ---------- 有有效二维码 ----------
        if not qr_state.get("lost", True):
            left, right, mode = self._normal_tracking(qr_state, obstacle_state)
            
            # 更新目标方向历史
            self.target_direction_history.append(target_direction)
            if len(self.target_direction_history) > self.max_history:
                self.target_direction_history.pop(0)
        
        # ---------- 丢失二维码 ----------
        else:
            print(f"[丢失] 已丢失 {qr_lost_timer}帧, 目标方向={target_direction:.1f}, 累积旋转={cumulative_rotation:.1f}度")
            
            # 检查是否有障碍物
            if obstacle_state.get("obstacle_near", False):
                print(f"[避障] 检测到障碍物，距离={obstacle_state['distance_cm']}cm")
                
                # 智能避障，同时保持方向
                left, right, mode = self._smart_avoidance(
                    obstacle_state, 
                    target_direction, 
                    cumulative_rotation
                )
            else:
                # 无障碍，朝目标方向搜索前进
                left, right, mode = self._search_toward_target(
                    target_direction, 
                    cumulative_rotation
                )
        
        # 保存最后的速度用于旋转估计
        self.last_left_speed = left*0.7
        self.last_right_speed = right
        
        # 创建返回结果
        result = {
            "left_speed": left,
            "right_speed": right,
            "mode": mode,
            "rotation_estimate": cumulative_rotation
        }
        
        # 记录模式
        self.last_mode = mode
        
        return result


if __name__ == "__main__":
    print("motion_controller 模块加载成功")