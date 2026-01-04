#!/usr/bin/env python3
# test_fix.py

from motion_controller import MotionController

def test_fix():
    print("=== 测试修复 ===")
    
    # 测试运动控制器
    controller = MotionController()
    
    # 模拟有二维码的情况
    qr_state = {
        "lost": False,
        "distance_cm": 50.0,
        "offsets": {
            "cx_off": 100.0,
            "cy_off": 0.0,
            "yaw_trend": 0.0,
            "pitch_trend": 0.0
        }
    }
    
    # 模拟无障碍状态
    obst_state = {
        "avoid_mode": False,
        "obstacle_near": False,
        "action": None
    }
    
    print("测试1: 有二维码，无障碍")
    result = controller.compute(qr_state, obst_state)
    print(f"  结果: 左轮={result['left_speed']:.3f}, 右轮={result['right_speed']:.3f}")
    
    # 测试丢失二维码
    qr_state_lost = {"lost": True}
    
    print("\n测试2: 丢失二维码")
    result = controller.compute(qr_state_lost, obst_state)
    print(f"  结果: 左轮={result['left_speed']:.3f}, 右轮={result['right_speed']:.3f}")
    
    print("\n测试完成！")

if __name__ == "__main__":
    test_fix()