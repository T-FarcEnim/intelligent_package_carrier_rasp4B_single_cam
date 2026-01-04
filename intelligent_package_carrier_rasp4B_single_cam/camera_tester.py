import RPi.GPIO as GPIO
import time
import sys
import tty
import termios
import threading
import numpy as np
import os
import subprocess

control_mode = "manual"  # manual/auto/avoidance/search

# 摄像头和二维码检测状态
qr_detection_enabled = False
camera_initialized = False
cap = None
camera_process = None  # ffplay进程对象


    

def init_camera():
    """初始化摄像头（仅在需要时动态加载OpenCV）"""
    global cap, camera_initialized, qr_detection_enabled
    
    # 检查是否启用二维码检测
    if not qr_detection_enabled:
        return False
    
    try:
        # 动态导入OpenCV和pyzbar
        import cv2
        from pyzbar import pyzbar
        
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("无法打开摄像头")
            return False
        
        # 设置较低分辨率以降低资源消耗
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        camera_initialized = True
        print("摄像头初始化成功")
        return True
    except Exception as e:
        print(f"摄像头初始化错误: {e}")
        return False
    
def stop_camera_preview():
    """停止摄像头预览终端"""
    global camera_process
    
    if camera_process and camera_process.poll() is None:
        camera_process.terminate()
        camera_process.wait(timeout=2)
        camera_process = None
        print("摄像头预览终端已停止")

def update_camera_preview():
    """根据当前模式更新摄像头预览状态"""
    global control_mode
    
    # 如果不是手动模式，启动摄像头预览
    if control_mode != "manual":
        # 检查是否已经开启了预览
        if camera_process is None or camera_process.poll() is not None:
            start_camera_preview()
    else:
        # 手动模式，停止摄像头预览
        stop_camera_preview()

def start_camera_preview():
    """启动摄像头预览终端（使用ffplay）"""
    global camera_process
    
    # 如果已有摄像头预览进程，先关闭它
    stop_camera_preview()
    
    try:
        # 检查ffplay是否可用
        result = subprocess.run(['which', 'ffplay'], capture_output=True, text=True)
        if result.returncode != 0:
            print("警告: ffplay未安装，无法启动摄像头预览")
            print("请安装ffmpeg: sudo apt-get install ffmpeg")
            return False
        
        print("正在启动摄像头预览终端...")
        
        # 启动ffplay摄像头预览（后台运行）
        camera_process = subprocess.Popen([
            'ffplay', 
            '-f', 'v4l2', 
            '-i', '/dev/video0',
            '-x', '320',   # 窗口宽度
            '-y', '240',   # 窗口高度
            '-window_title', '小车摄像头预览',
            '-noborder',   # 无边框
            '-alwaysontop' # 始终在最前
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        print("摄像头预览终端已启动")
        return True
    except Exception as e:
        print(f"启动摄像头预览失败: {e}")
        return False
    
def get_key():
    """获取单个按键输入（不需要按回车）"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main_control():
    """主控制函数"""
    global control_mode, qr_detection_enabled
    
    print("控制键:")
    print("  m: 手动模式    o: 自动模式    q:退")
    print("注意: 非手动模式下自动开启摄像头预览")
    print("提示: 自动模式有初始识别时间，请确保二维码在摄像头视野内")
    
    # 启动控制线程
    auto_thread = None
    avoidance_thread = None
    search_thread = None
    
    try:
        while True:
            key = get_key().lower()
            
            if key == 'q':
                print("退出程序")
                control_mode = "manual"
                stop_camera_preview()
                break
            elif key == 'm':
                control_mode = "manual"
                print("切换到手动模式")
                
                # 手动模式下关闭摄像头预览
                update_camera_preview()
            elif key == 'o':
                control_mode = "auto"
                print("切换到自动模式")
                # 非手动模式下启动摄像头预览
                update_camera_preview()
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        control_mode = "manual"
        stop_camera_preview()
        
        cleanup()

def cleanup():
    """清理资源"""
    global cap, camera_process
    
    # 停止摄像头预览
    stop_camera_preview()
    
    try:
        if cap and hasattr(cap, 'isOpened') and cap.isOpened():
            cap.release()
            print("摄像头已释放")
    except:
        pass

    print("资源清理完成")

if __name__ == "__main__":
    try:
        # 检查是否支持摄像头功能
        qr_detection_enabled = False
        if os.getenv('ENABLE_QR', '0') == '1':
            print("启用二维码检测功能")
            qr_detection_enabled = True
        
        main_control()
    except Exception as e:
        print(f"程序异常: {e}")
        cleanup()