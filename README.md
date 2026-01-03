# 自动循迹-避障智能小车系统

![GitHub License](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)
![Python Version](https://img.shields.io/badge/Python-3.9%2B-blue.svg)
![OpenCV Version](https://img.shields.io/badge/OpenCV-4.x-orange.svg)

基于树莓派4B（Debian 13）与OpenCV的单摄像头视觉跟踪系统，结合二维码定位与超声波避障功能，实现智能自主导航。

**项目地址**：<https://github.com/T-FarcEnim/intelligent_package_carrier_rasp4B_single_cam>

## 🎯 核心特性

- **视觉导航**：通过单摄像头识别二维码实现精准定位
- **智能避障**：超声波传感器实时检测障碍物，确保行驶安全
- **参数可配置**：所有运行参数通过JSON文件管理，无需修改代码
- **模块化设计**：功能模块清晰分离，便于维护和扩展
- **行为优先**：采用"目标优先+避障切换"策略，平衡效率与安全

## 📁 项目结构

```
intelligent_package_carrier_rasp4B_single_cam/
├── track_main.py              # 主程序：自动循迹
├── camera_module.py          # 摄像头管理模块
├── qr_detector.py           # 二维码检测与处理
├── obstacle_sensor.py       # 超声波避障传感器
├── motion_controller.py     # 运动控制逻辑
├── manual_control.py        # 手动控制测试程序
├── qr_scan_to_json.py       # 二维码扫描工具
├── LICENSE                  # MPL-2.0 许可证文件
├── README.md               # 项目说明文档
└── config/                  # 配置文件目录
    ├── camera_intrinsics.json  # 相机内参
    ├── qr_params.json        # 二维码参数
    └── speed_params.json     # 速度与行为参数
```

## ⚙️ 系统要求

### 硬件要求
- **主控制器**：树莓派4B (推荐4GB以上内存)
- **视觉传感器**：Raspberry Pi Camera Module v2 或兼容摄像头
- **避障传感器**：HC-SR04 超声波传感器
- **动力系统**：智能小车底盘（电机驱动需自行适配）

### 软件要求
- **操作系统**：Debian 13 (Raspberry Pi OS Bookworm)
- **Python版本**：Python 3.9+
- **核心库**：
  - OpenCV 4.x
  - OpenCV-contrib (含二维码检测模块)
  - NumPy

## 🔧 安装与配置

### 1. 安装依赖

```bash
# 更新系统软件包
sudo apt update && sudo apt upgrade -y

# 安装Python系统依赖
sudo apt install python3-pip python3-numpy python3-opencv -y

# 安装Python依赖包
pip3 install opencv-contrib-python
```

### 2. 克隆项目

```bash
git clone https://github.com/T-FarcEnim/intelligent_package_carrier_rasp4B_single_cam.git
cd intelligent_package_carrier_rasp4B_single_cam
```

### 3. 相机配置

启用树莓派摄像头接口：

```bash
sudo raspi-config
# 选择 Interface Options > Camera > Yes
# 重启树莓派：sudo reboot
```

## 🚀 快速开始

### 1. 测试二维码识别

首先测试摄像头和二维码识别功能：

```bash
python3 qr_scan_to_json.py
```

成功后将生成 `qr_scan_result.json` 文件，包含识别到的二维码信息。

### 2. 手动控制测试

测试电机驱动接口是否正常工作：

```bash
python3 manual_control.py
```

**控制按键**：
- `W`/`S`：前进/后退
- `A`/`D`：左转/右转
- `空格键`：停止
- `Q`：退出程序

### 3. 运行主程序

启动自动循迹-避障系统：

```bash
python3 track_main.py
```

## ⚙️ 参数配置

所有运行参数均在 `config/` 目录下的JSON文件中配置，修改后即时生效。

### `camera_intrinsics.json`
相机内参矩阵和畸变系数，影响距离计算精度。建议进行相机标定后更新此文件。

```json
{
  "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
  "dist_coeffs": [k1, k2, p1, p2, k3],
  "resolution": [640, 480]
}
```

### `qr_params.json`
二维码识别相关参数：

```json
{
  "qr_size_cm": 2.5,
  "valid_codes": ["CAR001", "CAR002", "STATION1", "STATION2"],
  "max_tracking_distance": 150,
  "dead_zone_ratio": 0.1,
  "confidence_threshold": 0.7
}
```

### `speed_params.json`
运动控制参数：

```json
{
  "base_speed": 0.5,
  "turn_speed": 0.3,
  "min_speed": 0.1,
  "max_speed": 0.8,
  "distance_compensation": 0.01,
  "obstacle_avoidance": {
    "turn_duration": 1.0,
    "forward_duration": 0.5,
    "recovery_delay": 2.0,
    "safe_distance": 20.0
  }
}
```

## 🔍 调试流程

建议按以下顺序进行系统调试：

### 第一阶段：传感器测试
1. **摄像头测试**
   ```bash
   python3 -c "import cv2; cap = cv2.VideoCapture(0); print('摄像头状态:', cap.isOpened())"
   ```

2. **二维码识别测试**
   ```bash
   python3 qr_scan_to_json.py
   ```
   - 检查是否能稳定识别二维码
   - 验证距离计算准确性

3. **超声波传感器测试**
   ```python
   # 创建测试脚本 test_ultrasonic.py
   from obstacle_sensor import UltrasonicSensor
   sensor = UltrasonicSensor(trigger_pin=23, echo_pin=24)
   for i in range(10):
       print(f"距离测量 {i+1}: {sensor.distance():.2f} cm")
   ```

### 第二阶段：运动控制测试
1. **电机响应测试**
   ```bash
   python3 manual_control.py
   ```
   - 测试各个方向运动
   - 确认速度控制范围

2. **参数微调**
   - 调整 `speed_params.json` 中的速度参数
   - 测试不同距离下的跟踪效果

### 第三阶段：系统集成
1. **完整功能测试**
   ```bash
   python3 track_main.py
   ```
   - 观察循迹准确性
   - 测试避障反应

## 🛠️ 硬件接口适配

### 电机驱动接口

主程序中的 `drive_motor()` 函数需要根据实际硬件进行适配：

```python
def drive_motor(left_speed, right_speed):
    """
    驱动电机接口
    
    Args:
        left_speed: 左轮速度 (-1.0 到 1.0，负值为后退)
        right_speed: 右轮速度 (-1.0 到 1.0，负值为后退)
    """
    # 当前实现为打印输出
    print(f"[MOTOR] L: {left_speed:.2f}, R: {right_speed:.2f}")
    
    # ====== 根据实际硬件选择以下一种实现 ======
    
    # 方案1：GPIO + L298N驱动
    # left_motor.forward(abs(left_speed)) if left_speed >= 0 else left_motor.backward(abs(left_speed))
    # right_motor.forward(abs(right_speed)) if right_speed >= 0 else right_motor.backward(abs(right_speed))
    
    # 方案2：PCA9685 PWM控制
    # pwm_controller.set_pwm(LEFT_MOTOR_PIN, left_speed)
    # pwm_controller.set_pwm(RIGHT_MOTOR_PIN, right_speed)
    
    # 方案3：ROS电机控制
    # motor_msg.left_speed = left_speed
    # motor_msg.right_speed = right_speed
    # motor_pub.publish(motor_msg)
```

### 传感器引脚配置

在 `obstacle_sensor.py` 中修改超声波传感器引脚：

```python
# 默认引脚配置（BCM编号）
TRIGGER_PIN = 23  # GPIO23
ECHO_PIN = 24     # GPIO24
```

## 📊 系统行为说明

| 场景 | 系统反应 | 设计意图 |
|------|---------|----------|
| 识别到目标二维码 | 计算距离和偏移，调整行驶方向 | 实现精准目标跟踪 |
| 检测到前方障碍物 | 执行避障策略（右转→直行→左转） | 安全优先原则 |
| 短暂丢失二维码 | 保持最后已知状态，继续避障 | 防止失控和误动作 |
| 同时识别多个二维码 | 选择最近的一个作为目标 | 简化决策逻辑 |
| 光线剧烈变化 | 可能短暂丢失目标，自动尝试恢复 | 环境适应性设计 |
| 到达目标安全距离 | 减速并在设定距离停止 | 防止碰撞目标 |

## 🔧 维护与扩展

### 添加新传感器
```python
# 示例：添加红外传感器
class IRSensor:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(pin, GPIO.IN)
    
    def detect_obstacle(self):
        return GPIO.input(self.pin) == GPIO.LOW

# 集成到避障系统
def check_all_sensors(self):
    ultrasonic_distance = self.ultrasonic.distance()
    ir_detected = self.ir_sensor.detect_obstacle()
    return ultrasonic_distance < SAFE_DISTANCE or ir_detected
```

### 日志记录配置
启用详细日志记录以便调试：

```python
import logging
import sys

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('car_system.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)
```

## 🐛 故障排除

### 常见问题及解决方案

| 问题 | 可能原因 | 解决方案 |
|------|---------|----------|
| 摄像头无法打开 | 1. 摄像头未启用<br>2. 被其他进程占用 | 1. 运行 `sudo raspi-config` 启用摄像头<br>2. 重启树莓派或关闭其他程序 |
| 二维码识别率低 | 1. 光照不足<br>2. 二维码尺寸太小<br>3. 摄像头对焦不准 | 1. 改善照明条件<br>2. 增大二维码尺寸<br>3. 调整摄像头焦距 |
| 距离测量不准 | 1. 相机内参不准确<br>2. 二维码实际尺寸设置错误 | 1. 重新进行相机标定<br>2. 精确测量二维码实际尺寸 |
| 避障反应迟钝 | 1. 超声波传感器故障<br>2. 检测频率过低 | 1. 检查传感器连接<br>2. 调整检测间隔时间 |
| 电机不响应 | 1. 驱动电路故障<br>2. 引脚配置错误<br>3. 电源不足 | 1. 检查电机驱动板<br>2. 确认GPIO引脚映射<br>3. 检查电源电压和电流 |

### 调试工具
项目包含以下调试工具：

1. **视觉调试模式**：在 `camera_module.py` 中设置 `debug=True` 可显示实时画面
2. **数据记录**：运行时可选择保存传感器数据到CSV文件
3. **性能监控**：内置帧率计算和资源使用监控

## 📄 许可证

本项目采用 **Mozilla Public License 2.0 (MPL-2.0)** 许可证。

### MPL-2.0 许可证要点：
1. **文件级Copyleft**：修改本项目的源代码文件后，这些文件必须保持MPL-2.0开源
2. **组合自由**：可以将MPL授权的代码与其他许可证的代码组合，形成更大的作品
3. **专利授权**：贡献者授予用户专利使用权
4. **明确责任**：代码按"原样"提供，作者不承担任何责任

### 使用要求：
- 使用、复制、修改本项目代码时，必须保留原许可证声明
- 分发修改版本时，必须在修改的文件中明确说明更改内容
- 可以将本项目代码用于商业闭源产品，但修改的文件仍需开源

完整许可证文本请查看 [LICENSE](LICENSE) 文件。

## 🤝 贡献指南

欢迎对项目进行改进和扩展：

1. **报告问题**：在GitHub Issues中描述遇到的问题
2. **功能建议**：提出改进建议或新功能想法
3. **提交代码**：通过Pull Request提交改进
   - 确保代码符合PEP 8规范
   - 添加必要的注释和文档
   - 更新相关的配置文件示例

## 📞 支持与联系

- **项目主页**：<https://github.com/T-FarcEnim/intelligent_package_carrier_rasp4B_single_cam>
- **问题反馈**：通过GitHub Issues提交问题
- **讨论区**：项目Wiki页面包含更多技术细节

## 📈 版本历史

NaN

---

**重要提示**：首次运行前请确保所有硬件连接正确，建议在安全、开阔的环境中进行测试。开发过程中请遵守实验室安全规范。