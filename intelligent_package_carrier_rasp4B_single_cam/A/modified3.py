import RPi.GPIO as GPIO
import time
import sys
import tty
import termios
import threading
import numpy as np
import os
import subprocess

# 电机引脚定义
LEFT_IN1 = 16
LEFT_IN2 = 19
RIGHT_IN3 = 20
RIGHT_IN4 = 21

# 超声波引脚定义
TRIG = 27
ECHO = 22

# PWM频率
PWM_FREQ = 1000  # 1kHz

# 控制参数
SAFE_DISTANCE = 30.0  # 安全距离 (cm)
TARGET_DISTANCE = 50.0  # 目标跟随距离 (cm)
QR_SIZE_THRESHOLD = 100  # 二维码尺寸阈值 (像素)
SPEED_LIMIT = 70  # 最大速度限制 (%)
MAX_QR_LOST_COUNT = 10  # 增加二维码最大丢失容忍次数（约1秒）
INITIALIZATION_TIME = 2.0  # 初始识别时间（秒）
SEARCH_STEPS = 3  # 搜索步骤数（向左/向右转动次数）
SEARCH_ANGLE = 30  # 每次搜索转动的角度（通过速度和时间控制）

# 特定二维码内容（您需要修改为您的二维码内容）
TARGET_QR_DATA = "0001"  # 替换为您的特定二维码内容

# 状态变量
current_speed = 80  # 初始速度 50% (0-100)
speed_step = 10     # 每次速度调整的步长
control_mode = "manual"  # manual/auto/avoidance/search
motor_vector = [0, 0]  # [左轮速度, 右轮速度] (-100 到 100)

# 手动模式复合运动状态
manual_base_speed = 0  # 基础速度（前进为正，后退为负）
manual_turn_bias = 0   # 转向偏置（左转为负，右转为正）

# 二维码跟踪状态变量
last_valid_qr_left = 0  # 上一次有效的左轮速度
last_valid_qr_right = 0  # 上一次有效的右轮速度
qr_lost_count = 0  # 二维码丢失计数器
search_step = 0  # 搜索步骤计数器
search_direction = 1  # 搜索方向：1向右，-1向左
target_qr_detected = False  # 是否检测到目标二维码
last_qr_center_x = 160  # 上一次二维码中心位置（默认图像中心）
initialization_complete = False  # 初始识别是否完成

# PWM对象
left_pwm1 = None
left_pwm2 = None
right_pwm1 = None
right_pwm2 = None

# 摄像头和二维码检测状态
qr_detection_enabled = False
camera_initialized = False
cap = None
camera_process = None  # ffplay进程对象

def init_gpio():
    """初始化GPIO和PWM"""
    global left_pwm1, left_pwm2, right_pwm1, right_pwm2
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # 设置电机引脚为输出模式
    for pin in [LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4]:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    
    # 设置超声波引脚
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.output(TRIG, GPIO.LOW)
    
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

def start_camera_preview():
    stop_camera_preview()
    '''
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
        '''

def stop_camera_preview():
    """停止摄像头预览终端"""
    global camera_process
    
    if camera_process and camera_process.poll() is None:
        camera_process.terminate()
        camera_process.wait(timeout=2)
        camera_process = None
        print("摄像头预览终端已停止")

def set_motor_vector(left, right):
    """使用二维向量设置电机速度，范围 -100 到 100"""
    global motor_vector
    
    motor_vector = [left, right]
    
    # 左电机
    if left >= 0:
        left_pwm1.ChangeDutyCycle(min(left, 100))
        left_pwm2.ChangeDutyCycle(0)
    else:
        left_pwm1.ChangeDutyCycle(0)
        left_pwm2.ChangeDutyCycle(min(-left, 100))
    
    # 右电机
    if right >= 0:
        right_pwm1.ChangeDutyCycle(min(right, 100))
        right_pwm2.ChangeDutyCycle(0)
    else:
        right_pwm1.ChangeDutyCycle(0)
        right_pwm2.ChangeDutyCycle(min(-right, 100))

def apply_manual_control():
    """应用手动模式的复合运动控制"""
    global manual_base_speed, manual_turn_bias
    
    # 计算左右轮速度
    left_speed = manual_base_speed - manual_turn_bias
    right_speed = manual_base_speed + manual_turn_bias
    
    # 速度限幅
    left_speed = np.clip(left_speed, -current_speed, current_speed)
    right_speed = np.clip(right_speed, -current_speed, current_speed)
    
    set_motor_vector(left_speed, right_speed)
    
    # 打印状态
    direction = "前进" if manual_base_speed > 0 else "后退" if manual_base_speed < 0 else "停止"
    turn = "左转" if manual_turn_bias > 0 else "右转" if manual_turn_bias < 0 else "直行"
    print(f"手动控制: {direction} + {turn}, 速度: [{left_speed:.1f}, {right_speed:.1f}]")

def move_forward(speed=None):
    """前进"""
    global manual_base_speed, manual_turn_bias
    speed = speed if speed is not None else current_speed
    manual_base_speed = speed
    apply_manual_control()

def move_backward(speed=None):
    """后退"""
    global manual_base_speed, manual_turn_bias
    speed = speed if speed is not None else current_speed
    manual_base_speed = -speed
    apply_manual_control()

def turn_left(speed=None):
    """左转（复合运动）"""
    global manual_turn_bias
    speed = speed if speed is not None else current_speed
    
    # 切换转向状态：如果已经在左转，则恢复直行
    if manual_turn_bias > 0:
        manual_turn_bias = 0
        print("恢复直行")
    else:
        manual_turn_bias = speed * 0.5  # 转向偏置为速度的一半
        print("左转")
    
    apply_manual_control()

def turn_right(speed=None):
    """右转（复合运动）"""
    global manual_turn_bias
    speed = speed if speed is not None else current_speed
    
    # 切换转向状态：如果已经在右转，则恢复直行
    if manual_turn_bias < 0:
        manual_turn_bias = 0
        print("恢复直行")
    else:
        manual_turn_bias = -speed * 0.5  # 转向偏置为速度的一半
        print("右转")
    
    apply_manual_control()

def stop():
    """停止"""
    global manual_base_speed, manual_turn_bias
    manual_base_speed = 0
    manual_turn_bias = 0
    print("停止")
    set_motor_vector(0, 0)

def increase_speed():
    """增加速度"""
    global current_speed
    current_speed = min(100, current_speed + speed_step)
    print(f"速度增加至: {current_speed}%")
    
    # 如果正在移动，更新速度
    if manual_base_speed != 0:
        manual_base_speed = current_speed if manual_base_speed > 0 else -current_speed
        apply_manual_control()

def decrease_speed():
    """降低速度"""
    global current_speed
    current_speed = max(0, current_speed - speed_step)
    print(f"速度降低至: {current_speed}%")
    
    # 如果正在移动，更新速度
    if manual_base_speed != 0:
        manual_base_speed = current_speed if manual_base_speed > 0 else -current_speed
        apply_manual_control()

def set_speed(speed_percent):
    """直接设置速度"""
    global current_speed
    if 0 <= speed_percent <= 100:
        current_speed = speed_percent
        print(f"速度设置为: {current_speed}%")
        
        # 如果正在移动，更新速度
        if manual_base_speed != 0:
            manual_base_speed = current_speed if manual_base_speed > 0 else -current_speed
            apply_manual_control()
    else:
        print("速度必须在 0 到 100 之间")

def get_distance():
    """使用超声波测量距离 (cm)"""
    try:
        # 确保Trig为低电平
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.01)
        
        # 发送10us的脉冲
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG, GPIO.LOW)
        
        # 等待回波开始
        start_time = time.time()
        while GPIO.input(ECHO) == GPIO.LOW:
            if time.time() - start_time > 0.1:  # 超时处理
                return float("inf")
        
        # 记录回波开始时间
        pulse_start = time.time()
        
        # 等待回波结束
        start_time = time.time()
        while GPIO.input(ECHO) == GPIO.HIGH:
            if time.time() - start_time > 0.1:  # 超时处理
                return float("inf")
        
        pulse_end = time.time()
        
        # 计算距离 (cm)
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # 声速343m/s = 17150 cm/s
        
        # 有效距离范围 2cm - 400cm
        return max(2.0, min(distance, 400.0))
    
    except Exception as e:
        print(f"测距错误: {e}")
        return float("inf")

def detect_qr_code():
    """检测二维码并返回尺寸、位置和内容信息（动态加载OpenCV）"""
    global cap, camera_initialized
    
    if not qr_detection_enabled:
        return None, None, None, None
    
    try:
        # 动态导入OpenCV和pyzbar
        import cv2
        from pyzbar import pyzbar
        
        # 检查摄像头是否已初始化
        if not camera_initialized:
            if not init_camera():
                return None, None, None, None
        
        ret, frame = cap.read()
        if not ret:
            return None, None, None, None
        
        # 转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 检测二维码
        barcodes = pyzbar.decode(gray)
        
        qr_data = None
        qr_size = None
        qr_center_x = None
        qr_center_y = None
        
        for barcode in barcodes:
            # 获取二维码数据
            barcode_data = barcode.data.decode("utf-8")
            
            # 只处理目标二维码
            if barcode_data == TARGET_QR_DATA:
                # 获取二维码边界框
                (x, y, w, h) = barcode.rect
                
                # 计算中心点
                center_x = x + w // 2
                center_y = y + h // 2
                
                # 计算二维码尺寸 (取宽高中的最大值)
                size = max(w, h)
                
                qr_data = barcode_data
                qr_size = size
                qr_center_x = center_x
                qr_center_y = center_y
                break  # 找到目标二维码后停止搜索
        
        return qr_size, qr_center_x, qr_center_y, qr_data
    
    except Exception as e:
        print(f"二维码检测错误: {e}")
        return None, None, None, None

def avoidance_behavior(distance):
    """避障行为控制（延长动作时间确保有效避障）"""
    if distance < SAFE_DISTANCE / 2:
        # 非常接近障碍物，快速后退更长时间
        print("紧急避障：快速后退")
        move_backward(min(80, current_speed + 20))
        time.sleep(0.8)  # 延长后退时间
    elif distance < SAFE_DISTANCE:
        # 接近障碍物，后退更长时间
        print("避障：后退")
        move_backward()
        time.sleep(0.5)  # 延长后退时间
    else:
        # 安全距离内，随机转向更长时间
        if np.random.rand() > 0.5:
            print("避障：左转")
            turn_left()
        else:
            print("避障：右转")
            turn_right()
        time.sleep(0.4)  # 延长转向时间

def qr_search_behavior():
    """二维码搜索行为：根据超声波检测选择较远方向搜索"""
    global search_step
    
    print(f"开始二维码搜索，步骤 {search_step+1}/{SEARCH_STEPS*2}")
    
    # 1. 测量前方距离
    front_distance = get_distance()
    print(f"前方距离: {front_distance:.1f} cm")
    
    # 2. 测量左侧距离（通过短暂左转）
    print("测量左侧距离...")
    set_motor_vector(-30, 30)  # 左转
    time.sleep(0.3)
    left_distance = get_distance()
    set_motor_vector(0, 0)
    time.sleep(0.2)
    
    # 3. 测量右侧距离（通过短暂右转）
    print("测量右侧距离...")
    set_motor_vector(30, -30)  # 右转
    time.sleep(0.3)
    right_distance = get_distance()
    set_motor_vector(0, 0)
    time.sleep(0.2)
    
    print(f"左侧距离: {left_distance:.1f} cm, 右侧距离: {right_distance:.1f} cm")
    
    # 4. 选择障碍物较少的一侧
    if left_distance > right_distance and left_distance > SAFE_DISTANCE:
        # 向左转
        print("选择向左转搜索")
        set_motor_vector(-40, 40)  # 左转
        time.sleep(0.5)
        # 前进一小段
        set_motor_vector(40, 40)
        time.sleep(0.8)
        # 转回原方向
        set_motor_vector(40, -40)  # 右转
        time.sleep(0.5)
    elif right_distance > SAFE_DISTANCE:
        # 向右转
        print("选择向右转搜索")
        set_motor_vector(40, -40)  # 右转
        time.sleep(0.5)
        # 前进一小段
        set_motor_vector(40, 40)
        time.sleep(0.8)
        # 转回原方向
        set_motor_vector(-40, 40)  # 左转
        time.sleep(0.5)
    else:
        # 两侧都有障碍物，后退
        print("两侧都有障碍物，后退")
        set_motor_vector(-40, -40)
        time.sleep(1.0)
    
    set_motor_vector(0, 0)
    time.sleep(0.5)
    
    # 更新搜索步骤
    search_step += 1
    if search_step >= SEARCH_STEPS * 2:
        search_step = 0
        print("完成一轮搜索，重新开始")

def qr_tracking_behavior(qr_size, qr_center_x, qr_center_y):
    """二维码跟踪行为控制（改进版）"""
    global last_valid_qr_left, last_valid_qr_right, last_qr_center_x
    
    # 获取图像中心
    img_center_x = 320  # 假设图像宽度为320
    img_center_y = 240  # 假设图像高度为240
    
    # 计算二维码位置偏差（水平和垂直）
    position_error_x = qr_center_x - img_center_x
    position_error_y = img_center_y - qr_center_y  # 注意：图像坐标系Y轴向下为正
    
    # 计算二维码大小偏差
    size_error = qr_size - QR_SIZE_THRESHOLD
    
    # 比例控制参数
    Kp_position_x = 0.3  # 水平位置控制增益
    Kp_position_y = 0.2  # 垂直位置控制增益（控制前后距离）
    Kp_size = 0.5        # 大小控制增益
    
    # 基础速度 (基于大小偏差和垂直位置)
    # 当二维码在图像下半部分时，需要后退；在上半部分时，需要前进
    base_speed = Kp_size * size_error + Kp_position_y * position_error_y
    
    # 转向调整 (基于水平位置偏差)
    steer_adjust = Kp_position_x * position_error_x
    
    # 计算左右轮速度
    left_speed = base_speed - steer_adjust
    right_speed = base_speed + steer_adjust
    
    # 速度限幅
    max_speed = min(SPEED_LIMIT, current_speed)
    left_speed = np.clip(left_speed, -max_speed, max_speed)
    right_speed = np.clip(right_speed, -max_speed, max_speed)
    
    # 保存当前有效指令和位置
    last_valid_qr_left = left_speed
    last_valid_qr_right = right_speed
    last_qr_center_x = qr_center_x
    
    # 设置电机速度
    set_motor_vector(left_speed, right_speed)
    
    # 打印调试信息
    print(f"二维码跟踪: 尺寸={qr_size}, 位置=[{qr_center_x}, {qr_center_y}], 速度=[{left_speed:.1f}, {right_speed:.1f}]")

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

def auto_control_loop():
    """自动控制主循环（改进版）- 增加初始识别时间"""
    global control_mode, qr_lost_count, search_step, target_qr_detected, initialization_complete
    
    print("进入自动模式: 二维码跟踪 + 超声波避障")
    print(f"初始识别阶段: {INITIALIZATION_TIME}秒，请确保二维码在摄像头视野内")
    
    # 重置状态
    qr_lost_count = 0
    search_step = 0
    target_qr_detected = False
    initialization_complete = False
    
    # 更新摄像头预览状态（非手动模式，自动启动）
    update_camera_preview()
    
    # 1. 初始识别阶段：给摄像头足够的时间进行初始化
    start_time = time.time()
    init_qr_detected = False
    
    while time.time() - start_time < INITIALIZATION_TIME and control_mode == "auto":
        try:
            # 尝试检测二维码
            qr_size, qr_center_x, qr_center_y, qr_data = detect_qr_code()
            
            if qr_data is not None:
                print(f"初始阶段检测到目标二维码: {qr_data}")
                init_qr_detected = True
                qr_tracking_behavior(qr_size, qr_center_x, qr_center_y)
                break
            else:
                # 显示剩余时间
                remaining_time = INITIALIZATION_TIME - (time.time() - start_time)
                print(f"初始识别中... 剩余时间: {remaining_time:.1f}秒")
            
            time.sleep(0.2)
        except Exception as e:
            print(f"初始识别错误: {e}")
            time.sleep(0.2)
    
    initialization_complete = True
    
    if init_qr_detected:
        print("初始识别成功，开始正常跟踪")
    else:
        print(f"初始识别阶段结束，未检测到二维码，开始正常跟踪流程")
    
    # 2. 正常跟踪阶段
    while control_mode == "auto":
        try:
            # 1. 超声波避障检测（优先级低于二维码跟踪，但在初始阶段后生效）
            distance = get_distance()
            
            # 在初始化阶段后，才开始避障判断
            if initialization_complete:
                print(f"前方距离: {distance:.1f} cm") 
                
                # 紧急避障：如果距离太近，立即避障
                if distance < SAFE_DISTANCE / 2:
                    print("紧急避障！距离过近")
                    control_mode = "avoidance"
                    continue
            
            # 2. 二维码检测
            qr_size, qr_center_x, qr_center_y, qr_data = detect_qr_code()
            
            if qr_data is not None:
                # 检测到目标二维码：正常跟踪，重置丢失计数
                print(f"检测到目标二维码: {qr_data}")
                target_qr_detected = True
                qr_tracking_behavior(qr_size, qr_center_x, qr_center_y)
                qr_lost_count = 0
                search_step = 0
            else:
                # 未检测到目标二维码
                qr_lost_count += 1
                
                if qr_lost_count <= MAX_QR_LOST_COUNT and (last_valid_qr_left != 0 or last_valid_qr_right != 0):
                    # 短时间丢失且有历史指令：沿用上次有效指令
                    print(f"二维码丢失（{qr_lost_count}/{MAX_QR_LOST_COUNT}），沿用上次指令")
                    set_motor_vector(last_valid_qr_left, last_valid_qr_right)
                else:
                    # 长时间丢失或无历史指令：进入搜索模式
                    print("二维码长时间丢失，进入搜索模式")
                    control_mode = "search"
                    # 搜索模式完成后自动返回
                    break  # 退出自动循环，等待搜索完成
            
            time.sleep(0.1)
            
        except Exception as e:
            print(f"自动控制错误: {e}")
            stop()
            time.sleep(1)
    
    print("退出自动模式")

def search_control_loop():
    """搜索控制主循环 - 搜索完成后自动返回自动模式"""
    global control_mode, qr_lost_count, target_qr_detected
    
    print("进入二维码搜索模式")
    
    # 最大搜索尝试次数
    max_search_attempts = 5
    search_attempt = 0
    
    while control_mode == "search" and search_attempt < max_search_attempts:
        try:
            search_attempt += 1
            print(f"搜索尝试 {search_attempt}/{max_search_attempts}")
            
            # 执行搜索行为
            qr_search_behavior()
            
            # 搜索后检查是否重新检测到二维码
            qr_size, qr_center_x, qr_center_y, qr_data = detect_qr_code()
            
            if qr_data is not None:
                print("搜索成功！重新检测到二维码，返回自动模式")
                target_qr_detected = True
                control_mode = "auto"
                # 自动重启自动模式
                restart_auto_mode()
                return
            else:
                # 继续搜索
                print(f"未找到二维码，继续搜索... (尝试 {search_attempt}/{max_search_attempts})")
                
            time.sleep(0.5)
            
        except Exception as e:
            print(f"搜索控制错误: {e}")
            stop()
            time.sleep(1)
    
    # 如果最大搜索尝试次数用完还没找到二维码
    if search_attempt >= max_search_attempts:
        print("达到最大搜索尝试次数，停止搜索")
        stop()
    
    print("退出搜索模式")

def avoidance_control_loop():
    """避障控制主循环（改进版）- 避障完成后自动返回自动模式"""
    global control_mode
    
    print("进入避障模式")
    
    # 最大避障尝试次数
    max_avoidance_attempts = 5
    avoidance_attempt = 0
    
    while control_mode == "avoidance" and avoidance_attempt < max_avoidance_attempts:
        try:
            avoidance_attempt += 1
            
            # 获取距离
            distance = get_distance()
            print(f"避障距离: {distance:.1f} cm (尝试 {avoidance_attempt}/{max_avoidance_attempts})")
            
            # 执行避障行为
            avoidance_behavior(distance)
            
            # 检查是否可以返回自动模式
            if distance > SAFE_DISTANCE * 1.5:
                print("障碍物已避开，返回自动模式")
                control_mode = "auto"
                # 自动重启自动模式
                restart_auto_mode()
                return
            else:
                # 继续避障
                print("继续避障...")
        
        except Exception as e:
            print(f"避障控制错误: {e}")
            stop()
            time.sleep(1)
    
    # 如果最大避障尝试次数用完还没避开
    if avoidance_attempt >= max_avoidance_attempts:
        print("达到最大避障尝试次数，返回自动模式")
        control_mode = "auto"
        restart_auto_mode()
    
    print("退出避障模式")

def restart_auto_mode():
    """重启自动模式"""
    global control_mode
    
    print("正在重启自动模式...")
    
    # 短暂停顿，让状态稳定
    time.sleep(0.5)
    
    # 确保控制模式是自动
    control_mode = "auto"
    
    # 启动新的自动模式线程
    auto_thread = threading.Thread(target=auto_control_loop, daemon=True)
    auto_thread.start()
    
    print("自动模式已重启")

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
    
    print("=== 履带小车控制程序 ===")
    print("控制键:")
    print("  w: 前进        s: 后退")
    print("  a: 左转        d: 右转")
    print("  +: 加速        -: 减速")
    print("  空格: 停止     q: 退出")
    print("  m: 手动模式    o: 自动模式")
    print("  c: 切换二维码检测 (当前: {})".format("开启" if qr_detection_enabled else "关闭"))
    print(f"目标二维码: {TARGET_QR_DATA}")
    print(f"当前模式: {control_mode}, 速度: {current_speed}%")
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
                stop()
                # 手动模式下关闭摄像头预览
                stop_camera_preview()
            elif key == 'o':
                control_mode = "auto"
                print("切换到自动模式")
                # 非手动模式下启动摄像头预览
                
                # 启动自动控制线程
                if auto_thread is None or not auto_thread.is_alive():
                    auto_thread = threading.Thread(target=auto_control_loop, daemon=True)
                    auto_thread.start()
            elif key == 'c':
                qr_detection_enabled = not qr_detection_enabled
                print(f"二维码检测 {'已启用' if qr_detection_enabled else '已禁用'}")
            elif control_mode == "manual":
                if key == 'w':
                    move_forward()
                elif key == 's':
                    move_backward()
                elif key == 'a':
                    turn_left()
                elif key == 'd':
                    turn_right()
                elif key == ' ':
                    stop()
                elif key == '+' or key == '=':
                    increase_speed()
                elif key == '-' or key == '_':
                    decrease_speed()
                elif key in ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0']:
                    # 数字键直接设置速度（0-9对应0-90%）
                    speed_value = int(key) * 10
                    if key == '0':
                        speed_value = 100
                    set_speed(speed_value)
                else:
                    print(f"未知命令: {key}")
            
            # 当进入避障模式时，启动避障线程
            if control_mode == "avoidance" and (avoidance_thread is None or not avoidance_thread.is_alive()):
                # 非手动模式下启动摄像头预览
                update_camera_preview()
                avoidance_thread = threading.Thread(target=avoidance_control_loop, daemon=True)
                avoidance_thread.start()
            
            # 当进入搜索模式时，启动搜索线程
            if control_mode == "search" and (search_thread is None or not search_thread.is_alive()):
                # 非手动模式下启动摄像头预览
                update_camera_preview()
                search_thread = threading.Thread(target=search_control_loop, daemon=True)
                search_thread.start()
            
            # 检查是否自动模式线程已经结束但控制模式还是自动
            # 这种情况可能发生在搜索或避障模式后需要重新启动自动模式
            if control_mode == "auto" and (auto_thread is None or not auto_thread.is_alive()):
                print("检测到自动模式线程已结束，正在重启...")
                auto_thread = threading.Thread(target=auto_control_loop, daemon=True)
                auto_thread.start()
            
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        control_mode = "manual"
        stop_camera_preview()
        stop()
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
    
    if left_pwm1:
        left_pwm1.stop()
    if left_pwm2:
        left_pwm2.stop()
    if right_pwm1:
        right_pwm1.stop()
    if right_pwm2:
        right_pwm2.stop()
    
    GPIO.cleanup()
    print("资源清理完成")

if __name__ == "__main__":
    try:
        # 检查是否支持摄像头功能
        qr_detection_enabled = False
        if os.getenv('ENABLE_QR', '0') == '1':
            print("启用二维码检测功能")
            qr_detection_enabled = True
        
        init_gpio()
        main_control()
    except Exception as e:
        print(f"程序异常: {e}")
        cleanup()