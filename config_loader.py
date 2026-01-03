#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
config_loader.py
简单配置加载模块
负责读取：
1）相机内参文件
2）二维码尺寸参数文件
3）速度 / 向量 / PID 参数文件

所有 json 在运行时重新读取，便于在线调参
"""

import json
from pathlib import Path


class ConfigLoader:
    def __init__(self,
                 camera_json="config/camera_intrinsics.json",
                 qr_json="config/qr_params.json",
                 speed_json="config/speed_params.json"):

        self.camera_json = Path(camera_json)
        self.qr_json = Path(qr_json)
        self.speed_json = Path(speed_json)

    def _load_json(self, path: Path):
        if not path.exists():
            raise FileNotFoundError(f"配置文件缺失: {path}")
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    def load_camera_params(self):
        """
        返回:
        {
            "fx": float, "fy": float,
            "cx": float, "cy": float,
            "dist": [k1,k2,p1,p2,k3]
        }
        """
        return self._load_json(self.camera_json)

    def load_qr_params(self):
        """
        返回:
        {
            "qr_size_cm": 2.5,
            "valid_id_digits": 5,
            "qr_dict": {...}  # 允许的二维码内容映射
        }
        """
        return self._load_json(self.qr_json)

    def load_speed_params(self):
        """
        返回:
        {
            "base_forward": ...,
            "forward_comp": ...,
            "base_turn": ...,
            "turn_comp": ...,
            "pid": {...},
            "obstacle": {...}
        }
        """
        return self._load_json(self.speed_json)

    def load_all(self):
        """
        一次性读取全部参数
        """
        return {
            "camera": self.load_camera_params(),
            "qr": self.load_qr_params(),
            "speed": self.load_speed_params()
        }


if __name__ == "__main__":
    # 简单测试
    loader = ConfigLoader()
    all_cfg = loader.load_all()
    print("配置读取成功：")
    print(all_cfg.keys())