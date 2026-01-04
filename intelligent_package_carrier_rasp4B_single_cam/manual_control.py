#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
manual_control.py
手动控制运动模块（独立文件，不与自动模式耦合）

功能：
1）独立读取速度参数（json）
2）提供简单的电机控制接口
3）支持外部函数调用 + 可选键盘控制模式
4）不参与二维码跟踪与避障逻辑

若要接入真实电机，请替换 drive_motor_* 函数
"""

import time
import RPi.GPIO as GPIO
from config_loader import ConfigLoader

# =========================
# 树莓派GPIO引脚配置（BCM编号）
# =========================
LEFT_IN1 = 21
LEFT_IN2 = 20
RIGHT_IN3 = 19
RIGHT_IN4 = 16

PWM_FREQ = 1000  # PWM频率 1kHz

# PWM对象
left_pwm1 = None
left_pwm2 = None
right_pwm1 = None
right_pwm2 = None

# =========================
# GPIO初始化
# =========================
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
    
    print("GPIO初始化完成")

# =========================
# 真实电机驱动接口
# =========================
def drive_motor_left(speed):
    """驱动左电机，speed范围：-1.0 到 1.0"""
    # 确保GPIO已初始化
    if left_pwm1 is None:
        init_gpio()
    
    # 将速度（-1.0到1.0）转换为占空比（0-100）
    duty_cycle = abs(speed) * 100
    
    if speed >= 0:
        # 正转
        left_pwm1.ChangeDutyCycle(min(duty_cycle, 100))
        left_pwm2.ChangeDutyCycle(0)
    else:
        # 反转
        left_pwm1.ChangeDutyCycle(0)
        left_pwm2.ChangeDutyCycle(min(duty_cycle, 100))
    
    print(f"[MOTOR] LEFT = {speed:.2f} (占空比: {duty_cycle:.1f}%)")

def drive_motor_right(speed):
    """驱动右电机，speed范围：-1.0 到 1.0"""
    # 确保GPIO已初始化
    if right_pwm1 is None:
        init_gpio()
    
    # 将速度（-1.0到1.0）转换为占空比（0-100）
    duty_cycle = abs(speed) * 100
    
    if speed >= 0:
        # 正转
        right_pwm1.ChangeDutyCycle(min(duty_cycle, 100))
        right_pwm2.ChangeDutyCycle(0)
    else:
        # 反转
        right_pwm1.ChangeDutyCycle(0)
        right_pwm2.ChangeDutyCycle(min(duty_cycle, 100))
    
    print(f"[MOTOR] RIGHT = {speed:.2f} (占空比: {duty_cycle:.1f}%)")

def stop_all_motors():
    """停止所有电机"""
    if left_pwm1:
        left_pwm1.ChangeDutyCycle(0)
    if left_pwm2:
        left_pwm2.ChangeDutyCycle(0)
    if right_pwm1:
        right_pwm1.ChangeDutyCycle(0)
    if right_pwm2:
        right_pwm2.ChangeDutyCycle(0)
    print("[MOTOR] 所有电机已停止")

def cleanup_gpio():
    """清理GPIO资源"""
    if left_pwm1:
        left_pwm1.stop()
    if left_pwm2:
        left_pwm2.stop()
    if right_pwm1:
        right_pwm1.stop()
    if right_pwm2:
        right_pwm2.stop()
    
    GPIO.cleanup()
    print("GPIO资源已清理")


class ManualController:
    def __init__(self, speed_config="config/speed_params.json"):
        loader = ConfigLoader(speed_json=speed_config)
        cfg = loader.load_speed_params()

        self.max_speed = float(cfg.get("max_speed", 1.0))
        self.min_speed = float(cfg.get("min_speed", -1.0))
        self.step = float(cfg.get("manual_step", 0.1))

        self.l = 0.0
        self.r = 0.0
        
        # 初始化GPIO
        init_gpio()

    def _clamp(self, v):
        return max(self.min_speed, min(self.max_speed, v))

    def set_speed(self, left, right):
        self.l = self._clamp(left)
        self.r = self._clamp(right)

        drive_motor_left(self.l)
        drive_motor_right(self.r)

        return dict(left=self.l, right=self.r)

    def stop(self):
        return self.set_speed(0.0, 0.0)
    
    def cleanup(self):
        """清理控制器资源"""
        self.stop()
        cleanup_gpio()

    # ========= 键盘控制（可选） =========
    def interactive(self):
        """
        简单键盘控制：
            w: 前进
            s: 后退
            a: 左转
            d: 右转
            space: 停止
            q: 退出
        """
        print("手动控制模式：w/s/a/d, 空格停止, q退出")
        print(f"速度范围: [{self.min_speed}, {self.max_speed}], 步长: {self.step}")

        import sys
        import termios
        import tty

        def getch():
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            return ch

        while True:
            key = getch()

            if key == "w":
                # 前进：左右轮同时加速
                self.set_speed(self.l + self.step, self.r + self.step)

            elif key == "s":
                # 后退：左右轮同时减速
                self.set_speed(self.l - self.step, self.r - self.step)

            elif key == "a":
                # 左转：左轮减速，右轮加速
                self.set_speed(self.l - self.step, self.r + self.step)

            elif key == "d":
                # 右转：左轮加速，右轮减速
                self.set_speed(self.l + self.step, self.r - self.step)

            elif key == " ":
                # 停止
                self.stop()

            elif key == "q":
                # 退出
                self.stop()
                print("退出手动控制")
                break

            time.sleep(0.05)


if __name__ == "__main__":
    try:
        mc = ManualController()
        mc.interactive()
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        if 'mc' in locals():
            mc.cleanup()