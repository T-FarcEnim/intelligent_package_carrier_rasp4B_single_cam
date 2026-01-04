import RPi.GPIO as GPIO
import time
import sys
import tty
import termios
import threading
import numpy as np
import os
import subprocess
import atexit

# 全局控制状态
control_mode = "manual"  # manual/auto/avoidance/search
tracking_active = False  # 二维码跟踪状态
target_qr_code = None    # 要跟踪的目标二维码内容

# 摄像头管理状态
qr_detection_enabled = False
camera_in_use_by = None  # 摄像头当前使用者: None, "preview", "tracking"
cap = None               # OpenCV摄像头对象
camera_process = None    # ffplay预览进程对象
preview_pid = None       # ffplay进程PID（用于强制终止）

# 注册退出处理
atexit.register(lambda: cleanup())

def ensure_camera_released():
    """确保摄像头设备完全释放"""
    import time
    
    def check_camera_free():
        """检查摄像头设备是否可用"""
        try:
            # 尝试以只读方式打开摄像头设备
            with open('/dev/video0', 'rb') as f:
                pass
            return True
        except IOError as e:
            # 设备被占用
            if "Device or resource busy" in str(e):
                return False
            # 其他错误（如不存在）
            return True
    
    # 等待直到摄像头可用，最多3秒
    max_wait = 3.0
    start_time = time.time()
    wait_count = 0
    
    while not check_camera_free():
        wait_count += 1
        elapsed = time.time() - start_time
        if elapsed > max_wait:
            print(f"警告: 摄像头资源释放超时 ({elapsed:.1f}秒)")
            return False
        
        # 每10次检查显示一次状态
        if wait_count % 10 == 0:
            print(f"等待摄像头释放... ({elapsed:.1f}秒)")
        
        time.sleep(0.1)  # 等待100ms再检查
    
    if wait_count > 0:
        print(f"摄像头释放完成，等待了 {wait_count*0.1:.1f} 秒")
    
    return True

def force_stop_camera_preview():
    """强制停止摄像头预览进程"""
    global camera_process, preview_pid, camera_in_use_by
    
    if camera_process:
        try:
            # 首先尝试正常终止
            if camera_process.poll() is None:
                camera_process.terminate()
                
                # 等待进程结束（最多2秒）
                try:
                    camera_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    # 如果进程不响应，强制终止
                    print("预览进程未响应，强制终止...")
                    camera_process.kill()
                    camera_process.wait()
        except Exception as e:
            print(f"停止预览进程时出错: {e}")
        finally:
            camera_process = None
            preview_pid = None
    
    # 如果有残留的预览进程，尝试通过PID终止
    if preview_pid:
        try:
            os.kill(preview_pid, 9)  # SIGKILL
            print(f"强制终止残留预览进程: PID {preview_pid}")
        except (OSError, ProcessLookupError):
            pass  # 进程已不存在
        preview_pid = None
    
    # 更新状态
    if camera_in_use_by == "preview":
        camera_in_use_by = None
    
    return True

def stop_camera_preview():
    """停止摄像头预览终端"""
    if not force_stop_camera_preview():
        return False
    
    # 确保摄像头资源释放
    time.sleep(0.2)  # 短暂等待确保内核释放资源
    ensure_camera_released()
    
    print("摄像头预览已停止")
    return True

def start_camera_preview():
    """启动摄像头预览终端（使用ffplay）"""
    global camera_process, preview_pid, camera_in_use_by
    
    # 检查是否已有预览进程
    if camera_process and camera_process.poll() is None:
        print("摄像头预览已在运行")
        return True
    
    # 检查摄像头是否被占用
    if camera_in_use_by == "tracking":
        print("摄像头正用于跟踪，无法启动预览")
        return False
    
    # 确保摄像头释放
    if not ensure_camera_released():
        print("无法获取摄像头资源")
        return False
    
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
            '-alwaysontop', # 始终在最前
            '-vf', 'format=yuv420p',  # 格式转换，兼容性更好
            '-loglevel', 'quiet'      # 减少日志输出
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        preview_pid = camera_process.pid
        
        # 更新状态
        camera_in_use_by = "preview"
        
        print(f"摄像头预览终端已启动 (PID: {preview_pid})")
        return True
    except Exception as e:
        print(f"启动摄像头预览失败: {e}")
        camera_in_use_by = None
        return False

def init_camera_for_tracking():
    """为跟踪初始化OpenCV摄像头"""
    global cap, camera_in_use_by
    
    # 检查摄像头是否已被占用
    if camera_in_use_by == "preview":
        print("摄像头正用于预览，请先停止预览")
        return False
    
    # 如果已经有OpenCV摄像头对象，先释放
    if cap is not None:
        try:
            cap.release()
            cap = None
        except:
            pass
    
    # 确保摄像头设备释放
    if not ensure_camera_released():
        print("摄像头资源无法释放，无法初始化跟踪")
        return False
    
    try:
        # 动态导入OpenCV和pyzbar
        import cv2
        from pyzbar import pyzbar
        
        print("正在初始化摄像头用于跟踪...")
        
        # 尝试打开摄像头（最多尝试3次）
        for attempt in range(3):
            try:
                # 优先使用V4L2驱动
                cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
                if not cap.isOpened():
                    # 如果失败，尝试默认驱动
                    cap = cv2.VideoCapture(0)
                
                if cap.isOpened():
                    break
                    
            except Exception as e:
                print(f"摄像头打开尝试 {attempt+1} 失败: {e}")
                time.sleep(0.5)  # 等待后重试
        
        if not cap or not cap.isOpened():
            print("无法打开摄像头进行跟踪")
            return False
        
        # 设置摄像头参数
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        cap.set(cv2.CAP_PROP_FPS, 15)
        
        # 更新状态
        camera_in_use_by = "tracking"
        print("跟踪摄像头初始化成功")
        return True
        
    except ImportError as e:
        print(f"缺少依赖库: {e}")
        print("请安装: sudo pip3 install opencv-python pyzbar")
        return False
    except Exception as e:
        print(f"跟踪摄像头初始化失败: {e}")
        camera_in_use_by = None
        return False

def update_camera_preview():
    """根据当前模式更新摄像头预览状态"""
    global control_mode, tracking_active
    
    # 如果是手动模式，停止所有摄像头活动
    if control_mode == "manual":
        if tracking_active:
            stop_qr_tracking()
        stop_camera_preview()
        return
    
    # 如果是自动模式且有跟踪，不启动预览
    if control_mode == "auto" and tracking_active:
        return
    
    # 自动模式且没有跟踪，启动预览
    if control_mode == "auto" and not tracking_active:
        start_camera_preview()

def decode_qr_data(qr_data):
    """安全解码二维码数据，尝试多种编码"""
    encodings_to_try = ['utf-8', 'gbk', 'gb2312', 'big5', 'latin-1', 'ascii']
    
    for encoding in encodings_to_try:
        try:
            decoded = qr_data.decode(encoding)
            # 检查是否包含很多问号（可能解码不正确）
            if '?' in decoded and encoding != 'latin-1':
                # 问号过多可能不是正确编码，继续尝试
                if decoded.count('?') / len(decoded) > 0.3:
                    continue
            return decoded, encoding
        except UnicodeDecodeError:
            continue
        except Exception:
            continue
    
    # 如果所有编码都失败，使用latin-1（不会失败，但可能不是正确显示）
    try:
        return qr_data.decode('latin-1'), 'latin-1'
    except:
        # 最后手段：转换为十六进制字符串
        return qr_data.hex(), 'hex'

def qr_tracking_thread():
    """二维码跟踪线程函数"""
    global control_mode, tracking_active, target_qr_code, cap
    global camera_in_use_by
    
    print("二维码跟踪线程启动")
    
    # 初始化摄像头用于跟踪
    if not init_camera_for_tracking():
        print("摄像头初始化失败，无法进行二维码跟踪")
        tracking_active = False
        return
    
    # 动态导入OpenCV和pyzbar
    try:
        import cv2
        from pyzbar import pyzbar
    except ImportError as e:
        print(f"导入OpenCV或pyzbar失败: {e}")
        tracking_active = False
        return
    
    frame_count = 0
    tracking_window_created = False
    last_qr_position = None  # 记录上次二维码位置，用于平滑
    
    while control_mode == "auto" and tracking_active:
        if cap is None or not cap.isOpened():
            print("摄像头未就绪，等待...")
            time.sleep(0.1)
            continue
        
        # 读取摄像头帧
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头帧")
            time.sleep(0.1)
            continue
        
        frame_count += 1
        
        # 每5帧处理一次，平衡性能和实时性
        if frame_count % 5 != 0:
            continue
        
        # 转换为灰度图像以提高处理速度
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        try:
            # 检测二维码
            qr_codes = pyzbar.decode(gray)
        except Exception as e:
            print(f"二维码解码错误: {e}")
            continue
        
        # 寻找目标二维码
        target_found = False
        target_center_x = 0
        target_center_y = 0
        current_qr_data = None
        qr_rect = None
        
        for qr in qr_codes:
            try:
                # 获取二维码位置（无论解码是否成功都要获取）
                (x, y, w, h) = qr.rect
                qr_rect = (x, y, w, h)
                
                # 计算中心点
                center_x = x + w // 2
                center_y = y + h // 2
                
                # 尝试解码数据
                qr_data = None
                encoding_used = 'unknown'
                
                if hasattr(qr, 'data') and qr.data:
                    decoded_data, encoding_used = decode_qr_data(qr.data)
                    qr_data = decoded_data
                    
                    # 打印解码信息（调试用）
                    if frame_count % 25 == 0:  # 每25次处理打印一次
                        print(f"解码二维码: {qr_data[:30]}... (编码: {encoding_used})")
                
                # 检查是否为目标二维码
                is_target = False
                if target_qr_code is None:
                    is_target = True  # 未指定目标，跟踪所有二维码
                elif qr_data and qr_data == target_qr_code:
                    is_target = True
                
                if is_target:
                    target_found = True
                    target_center_x = center_x
                    target_center_y = center_y
                    current_qr_data = qr_data
                    
                    # 保存位置用于平滑
                    last_qr_position = (center_x, center_y, w, h)
                    
                    # ========== 绘制绿色框和红色点 ==========
                    # 绘制绿色矩形框
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # 绘制红色中心点
                    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                    
                    # 在中心点添加十字标记
                    cross_size = 8
                    cv2.line(frame, (center_x - cross_size, center_y), 
                            (center_x + cross_size, center_y), (0, 0, 255), 2)
                    cv2.line(frame, (center_x, center_y - cross_size), 
                            (center_x, center_y + cross_size), (0, 0, 255), 2)
                    
                    # ========== 显示二维码信息 ==========
                    if qr_data:
                        # 截断过长的文本
                        display_text = qr_data
                        if len(display_text) > 20:
                            display_text = display_text[:17] + "..."
                        
                        # 添加编码信息
                        if encoding_used != 'utf-8' and encoding_used != 'hex':
                            display_text += f" [{encoding_used}]"
                        
                        # 绘制文本背景（提高可读性）
                        text_size = cv2.getTextSize(display_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                        cv2.rectangle(frame, (x, y - text_size[1] - 10), 
                                    (x + text_size[0] + 10, y), (0, 0, 0), -1)
                        
                        # 绘制文本
                        cv2.putText(frame, display_text, (x + 5, y - 5), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # 显示二维码类型
                    qr_type = getattr(qr, 'type', 'UNKNOWN')
                    type_text = f"Type: {qr_type}"
                    cv2.putText(frame, type_text, (x, y + h + 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    
                    break  # 找到目标二维码后跳出循环
            
            except Exception as e:
                print(f"处理单个二维码时出错: {e}")
                continue
        
        # ========== 绘制图像中心十字线 ==========
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2
        
        # 绘制蓝色十字准线
        line_length = 25
        cv2.line(frame, (frame_center_x - line_length, frame_center_y), 
                (frame_center_x + line_length, frame_center_y), (255, 0, 0), 2)
        cv2.line(frame, (frame_center_x, frame_center_y - line_length), 
                (frame_center_x, frame_center_y + line_length), (255, 0, 0), 2)
        
        # 在中心点绘制圆点
        cv2.circle(frame, (frame_center_x, frame_center_y), 8, (255, 0, 0), 2)
        
        # ========== 显示跟踪状态信息 ==========
        # 状态栏背景
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 70), (0, 0, 0), -1)
        
        if target_found:
            # 计算偏移量
            offset_x = target_center_x - frame_center_x
            offset_y = target_center_y - frame_center_y
            
            # 显示跟踪状态
            status_text = "状态: 跟踪中"
            cv2.putText(frame, status_text, (10, 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 显示偏移量
            offset_text = f"偏移量: X={offset_x:+04d}, Y={offset_y:+04d}"
            cv2.putText(frame, offset_text, (10, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # 显示二维码信息
            if current_qr_data:
                qr_info = f"二维码: {current_qr_data[:15]}..." if len(current_qr_data) > 15 else f"二维码: {current_qr_data}"
                cv2.putText(frame, qr_info, (250, 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 根据偏移量提供控制建议
            if abs(offset_x) > 30:
                direction = "右转" if offset_x > 0 else "左转"
                action = "前进" if abs(offset_x) < 50 else "快速转弯"
                control_text = f"控制: {direction} ({action})"
                cv2.putText(frame, control_text, (250, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # 未找到二维码
            status_text = "状态: 搜索中..."
            cv2.putText(frame, status_text, (10, 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            
            # 显示检测到的二维码数量
            qr_count = len(qr_codes)
            count_text = f"检测到: {qr_count} 个二维码"
            cv2.putText(frame, count_text, (10, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # 如果上次有二维码位置，显示最后已知位置
            if last_qr_position:
                last_text = "最后已知位置: 有记录"
                cv2.putText(frame, last_text, (250, 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        
        # 显示帧率信息
        fps_text = f"帧: {frame_count}"
        cv2.putText(frame, fps_text, (frame.shape[1] - 80, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        # 显示目标设置
        target_text = f"目标: {'所有' if target_qr_code is None else target_qr_code}"
        if len(target_text) > 20:
            target_text = target_text[:17] + "..."
        cv2.putText(frame, target_text, (frame.shape[1] - 150, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        # ========== 显示处理后的图像 ==========
        if tracking_active:
            window_title = "二维码跟踪"
            if target_qr_code:
                window_title += f" - 目标: {target_qr_code[:10]}..." if len(target_qr_code) > 10 else f" - 目标: {target_qr_code}"
            
            if not tracking_window_created:
                cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(window_title, 400, 350)
                tracking_window_created = True
            
            cv2.imshow(window_title, frame)
            
            # 等待按键，但保持实时处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' 或 ESC
                break
            elif key == ord('s'):  # 按s键停止跟踪
                stop_qr_tracking()
                break
    
    # 清理OpenCV窗口
    if tracking_window_created:
        cv2.destroyAllWindows()
        tracking_window_created = False
    
    # 释放摄像头资源
    if cap is not None:
        cap.release()
        cap = None
    
    # 更新状态
    if camera_in_use_by == "tracking":
        camera_in_use_by = None
    
    print("二维码跟踪线程结束")

def start_qr_tracking(qr_content=None):
    """开始二维码跟踪"""
    global tracking_active, target_qr_code, control_mode, camera_in_use_by
    
    if tracking_active:
        print("二维码跟踪已在运行")
        return False
    
    if control_mode != "auto":
        print("请在自动模式下启动跟踪 (按'o'键)")
        return False
    
    # 检查摄像头是否被占用
    if camera_in_use_by == "preview":
        print("正在停止摄像头预览...")
        if not stop_camera_preview():
            print("无法停止预览，跟踪无法开始")
            return False
    
    # 设置目标二维码
    target_qr_code = qr_content
    
    print("正在准备摄像头跟踪...")
    print("注意: 摄像头预览将暂时关闭")
    
    # 确保摄像头完全释放
    if not ensure_camera_released():
        print("摄像头资源无法释放，跟踪无法开始")
        return False
    
    # 启动跟踪
    tracking_active = True
    
    # 创建并启动跟踪线程
    tracking_thread = threading.Thread(target=qr_tracking_thread)
    tracking_thread.daemon = True
    tracking_thread.start()
    
    if qr_content:
        print(f"开始跟踪二维码: {qr_content}")
    else:
        print("开始跟踪所有二维码")
    
    return True

def stop_qr_tracking():
    """停止二维码跟踪"""
    global tracking_active, target_qr_code
    
    if not tracking_active:
        return True
    
    print("正在停止二维码跟踪...")
    tracking_active = False
    target_qr_code = None
    
    # 等待跟踪线程结束（最多1秒）
    time.sleep(0.5)
    
    # 如果是自动模式，重新开启预览
    if control_mode == "auto":
        print("跟踪已停止，恢复摄像头预览")
        update_camera_preview()
    
    return True

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
    
    print("=" * 50)
    print("小车摄像头控制系统")
    print("=" * 50)
    print("控制键:")
    print("  m: 手动模式    o: 自动模式")
    print("  t: 开始二维码跟踪    s: 停止跟踪")
    print("  p: 开启/关闭预览    q: 退出程序")
    print("-" * 50)
    print("注意:")
    print("  1. 自动模式下按't'开始跟踪，预览将自动关闭")
    print("  2. 跟踪停止后，预览会自动恢复")
    print("  3. 手动模式下所有摄像头功能都会停止")
    print("=" * 50)
    
    try:
        while True:
            key = get_key().lower()
            
            if key == 'q':
                print("退出程序")
                control_mode = "manual"
                stop_qr_tracking()
                stop_camera_preview()
                break
            elif key == 'm':
                control_mode = "manual"
                stop_qr_tracking()
                print("切换到手动模式")
                print("所有摄像头功能已停止")
                
                # 手动模式下关闭所有摄像头活动
                update_camera_preview()
            elif key == 'o':
                control_mode = "auto"
                print("切换到自动模式")
                print("摄像头预览已启动")
                
                # 启动预览
                update_camera_preview()
            elif key == 't':
                if control_mode == "auto":
                    # 询问用户是否指定目标二维码
                    print("选择跟踪模式:")
                    print("  1: 跟踪所有二维码")
                    print("  2: 跟踪特定二维码")
                    print("请输入 1 或 2 (默认: 1): ", end='', flush=True)
                    
                    # 获取用户选择
                    choice = get_key()
                    print(choice)
                    
                    if choice == '2':
                        print("请输入要跟踪的二维码内容: ", end='', flush=True)
                        # 需要切换回正常终端模式来读取字符串
                        fd = sys.stdin.fileno()
                        old_settings = termios.tcgetattr(fd)
                        try:
                            termios.tcsetattr(fd, termios.TCSANOW, old_settings)
                            target = input()
                        finally:
                            tty.setraw(sys.stdin.fileno())
                        
                        if target:
                            start_qr_tracking(target)
                        else:
                            start_qr_tracking()
                    else:
                        start_qr_tracking()  # 跟踪所有二维码
                else:
                    print("请先切换到自动模式 (按'o'键)")
            elif key == 's':
                # 停止二维码跟踪
                stop_qr_tracking()
            elif key == 'p':
                # 切换预览状态
                if control_mode == "auto" and not tracking_active:
                    if camera_in_use_by == "preview":
                        print("关闭摄像头预览")
                        stop_camera_preview()
                    else:
                        print("开启摄像头预览")
                        start_camera_preview()
                else:
                    print("当前模式不支持手动切换预览")
            else:
                print(f"未知命令: {key}")
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        control_mode = "manual"
        stop_qr_tracking()
        stop_camera_preview()
        cleanup()

def cleanup():
    """清理资源"""
    global cap, camera_process
    
    print("正在清理资源...")
    
    # 停止二维码跟踪
    stop_qr_tracking()
    
    # 停止摄像头预览
    force_stop_camera_preview()
    
    # 释放OpenCV摄像头
    if cap is not None:
        try:
            cap.release()
            cap = None
            print("OpenCV摄像头已释放")
        except Exception as e:
            print(f"释放OpenCV摄像头时出错: {e}")
    
    # 清理OpenCV窗口
    try:
        import cv2
        cv2.destroyAllWindows()
    except:
        pass
    
    # 重置状态
    camera_in_use_by = None
    
    print("资源清理完成")

if __name__ == "__main__":
    try:
        # 检查是否支持摄像头功能
        qr_detection_enabled = False
        if os.getenv('ENABLE_QR', '0') == '1':
            print("启用二维码检测功能")
            qr_detection_enabled = True
        else:
            print("提示: 设置环境变量 ENABLE_QR=1 启用二维码检测")
        
        # 打印依赖检查
        print("检查依赖库...")
        try:
            import cv2
            from pyzbar import pyzbar
            print("  ✓ OpenCV 和 pyzbar 已安装")
        except ImportError as e:
            print(f"  ✗ 缺少依赖: {e}")
            print("  请安装: sudo pip3 install opencv-python pyzbar")
        
        # 检查ffplay
        result = subprocess.run(['which', 'ffplay'], capture_output=True, text=True)
        if result.returncode == 0:
            print("  ✓ ffplay 已安装")
        else:
            print("  ✗ ffplay 未安装")
            print("  请安装: sudo apt-get install ffmpeg")
        
        print("-" * 50)
        
        main_control()
    except Exception as e:
        print(f"程序异常: {e}")
        import traceback
        traceback.print_exc()
        cleanup()