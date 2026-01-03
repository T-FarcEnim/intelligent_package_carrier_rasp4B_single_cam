# 自动循迹–避障智能小车（树莓派4B · Debian 13 · OpenCV）

本系统基于单摄像头 + 二维码视觉跟踪 + 超声波避障。
运行逻辑采用“目标优先 + 避障切换 + 实时向量演算模拟 PID”。

> ⚠️ 本项目代码结构刻意保持简洁，便于理解与修改。  
> 所有运行参数存放在 `config/*.json` 中，运行时动态读取，不需要改代码即可调整行为。

---

## 📦 目录结构

project/
├── track_main.py            # 主控程序（跟踪 + 避障 + 运动）
├── camera_module.py         # 摄像头采集模块
├── qr_detector.py           # 二维码检测与姿态计算
├── obstacle_sensor.py       # 超声波避障模块
├── motion_controller.py     # 向量控制与运动决策
├── manual_control.py        # 手动控制（独立运行）
├── qr_scan_to_json.py       # 二维码扫描→结果写入 JSON
└── config/
    ├── camera_intrinsics.json
    ├── qr_params.json
    └── speed_params.json

---

## 🚗 系统工作流程（概述）

1. 摄像头采集图像（不启用预览窗口）
2. 识别画面中**属于字典的二维码**
3. 只选择**最近的二维码**作为目标
4. 根据：
   - 像素大小 → 计算距离
   - 角点像素位置 → 计算水平偏移与姿态
5. 结合超声波避障输入
6. 向量演算计算左右轮输出：
   - 直行速度补偿
   - 转向基础速度
   - 转向方向补偿
7. 模拟 PID 的实时响应
8. 底层驱动接口输出轮速（当前为占位函数）

丢失二维码时：
- 优先执行避障
- 无障碍则**保持上次状态与参数**直到再次识别成功

---

## 🧩 参数文件说明（运行时动态加载）

### `config/camera_intrinsics.json`
- 相机内参（标定后可替换）
- 影响距离计算与角点精度

### `config/qr_params.json`
- 二维码真实尺寸（默认 2.5 cm）
- 允许识别的二维码内容字典
- 跟踪距离上限 & 死区比例

### `config/speed_params.json`
- 直行/转弯速度基础值
- 远近距离补偿
- 上/下坡记忆逻辑
- 避障动作时序（右转→直行→左转→恢复）

> 修改 JSON 即可改变行为，无需改代码。

---

## ▶️ 运行方式

### 自动循迹（主程序）
```bash
python3 track_main.py

手动控制（测试电机接口用）

python3 manual_control.py

二维码采集 → 写入 JSON

python3 qr_scan_to_json.py

识别成功后生成：

qr_scan_result.json


⸻

🧪 建议调试流程
	1.	先运行 qr_scan_to_json.py
	•	确认能稳定识别二维码
	2.	调整 camera_intrinsics.json
	3.	调整 qr_params.json 中的尺寸与跟踪阈值
	4.	调整 speed_params.json（速度逐步提高）
	5.	最后再替换电机驱动接口

⸻

⚠️ 已知可能情况（行为说明）
	•	摄像头光照变化大 → 可能短暂丢失二维码
→ 系统会保持上次运动状态并等待恢复
	•	超声波误测过近值 → 会触发避障策略
	•	若二维码距离过近 → 车辆会减速并在安全距离内停止
	•	单摄像头距离估计存在误差 → 建议通过标定修正
	•	同时识别多个二维码 → 系统只选择最近的一个
	•	若完全丢失目标且无障碍 → 车辆保持上次方向慢行

以上行为属于设计预期，并非异常

⸻

🧱 后续对接电机驱动

当前 track_main.py 中：

def drive_motor(left, right):
    print("[DRIVE]", left, right)

替换为：
	•	GPIO + PWM / I2C 电机驱动
	•	ROS Topic / SPI 扩展板
	•	轮速需归一化到 speed_params.json 范围内

⸻

📌 维护建议
	•	优先修改 JSON，不直接改逻辑代码
	•	每次修改参数前先备份一个版本
	•	建议将运行日志写入文件以便调试

⸻

📝 许可与使用

本项目以教学与实验用途为主，
适用于嵌入式机器人课程、计算机视觉实验与自主小车研究。

