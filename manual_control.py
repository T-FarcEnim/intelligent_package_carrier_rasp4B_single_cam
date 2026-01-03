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
from config_loader import ConfigLoader


# =========================
#  占位电机驱动接口
#  （部署时替换为真实 GPIO / PWM 驱动）
# =========================
def drive_motor_left(speed):
    print(f"[MOTOR] LEFT = {speed:.2f}")


def drive_motor_right(speed):
    print(f"[MOTOR] RIGHT = {speed:.2f}")


class ManualController:
    def __init__(self, speed_config="config/speed_params.json"):
        loader = ConfigLoader(speed_json=speed_config)
        cfg = loader.load_speed_params()

        self.max_speed = float(cfg.get("max_speed", 1.0))
        self.min_speed = float(cfg.get("min_speed", -1.0))
        self.step = float(cfg.get("manual_step", 0.1))

        self.l = 0.0
        self.r = 0.0

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
                self.set_speed(self.l + self.step, self.r + self.step)

            elif key == "s":
                self.set_speed(self.l - self.step, self.r - self.step)

            elif key == "a":
                self.set_speed(self.l - self.step, self.r + self.step)

            elif key == "d":
                self.set_speed(self.l + self.step, self.r - self.step)

            elif key == " ":
                self.stop()

            elif key == "q":
                self.stop()
                print("退出手动控制")
                break

            time.sleep(0.05)


if __name__ == "__main__":
    mc = ManualController()
    mc.interactive()