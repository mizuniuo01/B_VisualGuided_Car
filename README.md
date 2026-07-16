# 视觉引导小车 — Visual Guided Car

基于 STM32F407 的视觉引导自主小车，通过 MaixCAM 视觉模块识别方向标志物与红绿灯，结合灰度传感器与超声波传感器，实现完整的环境感知与自主导航。

## 目录

- [硬件平台](#硬件平台)
- [软件架构](#软件架构)
- [构建与烧录](#构建与烧录)
- [目录结构](#目录结构)
- [功能特性](#功能特性)
- [许可证](#许可证)

---

## 硬件平台

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控 | STM32F407VET6 | Cortex-M4, 168MHz, FPU |
| 视觉模块 | MaixCAM | YOLOv5 推理 + 赛道扫线 (USART3) |
| 姿态传感器 | JY901S | Roll/Pitch/Yaw, 1ms 更新 (USART6) |
| 灰度传感器 | 感为科技 八路 | I2C2, 0x4C, 黑线检测 |
| 超声波 | HC-SR04 | TIM4 CH4 输入捕获, 障碍物检测 |
| 编码电机 | 轮趣科技 | 13PPR × 4 倍频 × 1:28 减速, QEI TIM1/2 |
| 电机驱动 | DRV8874 | PH/EN 模式, PWM TIM3 |
| 蓝牙 | 正点原子 ATK-BLE | USART1 DMA+IDLE, 仪表盘 + 调参 |

### 引脚分配

| 外设 | 引脚 | 模式 |
|------|------|------|
| TIM1 (左编码器) | PE9 CH1, PE11 CH2 | QEI |
| TIM2 (右编码器) | PA0 CH1, PA1 CH2 | QEI |
| TIM3 (PWM) | PA6 CH1, PA7 CH2 | PWM, ARR=8400 |
| TIM4 (超声波) | PD15 CH4 | 输入捕获 |
| TIM6 (系统tick) | — | 1ms 基础定时器 |
| USART1 (蓝牙) | PA9 TX, PA10 RX | DMA+IDLE |
| USART3 (视觉) | PB10 TX, PB11 RX | DMA+IDLE |
| USART6 (陀螺仪) | PC6 TX, PC7 RX | DMA+IDLE |
| I2C2 (灰度) | PB10 SCL, PB11 SDA | DMA, 100kHz |
| I2C3 (OLED) | PA8 SCL, PC9 SDA | 100kHz |
| GPIO 控制 | PA4/PA5/PC4/PC5 | 电机 nSLEEP/PH, 蜂鸣器 |

---

## 软件架构

```
control_manager     ← 顶层状态机 + 任务调度
    │
perception          ← 环境感知：视觉 + 灰度 + 超声波数据融合
    │
motion_manager      ← 运动管理：普通闭环 + 距离/角度运动规划
    │
motion_control      ← 底层闭环：角度环 + 双轮速度环 PID
    │
driver (motor/encoder/gyro/...)  ← 硬件驱动层
```

### 控制方案

- **分段前进** — 固定距离(685mm)分段时间驱动循环
- **方向补偿** — 坐标系状态机(前后左右0-3)自动修正转弯漂移(~10cm)
- **黑线逻辑** — 灰度量检测黑线 → 记录剩余距离 → 红绿灯判断 → 穿越路口
- **障碍物** — 超声波检测 → 立即停车 + 蜂鸣器 → 清除后恢复
- **STOP 标志** — YOLO 检测 STOP 标志 → 当前段完成后停车

### 闭环控制

- **角度环** — 绝对 yaw 锁定(unwrap + target align), PID 微分-on-实际值
- **速度环** — 左右轮独立编码器反馈, 共享 PID 参数

### 调度机制

TIM6 1ms ISR → 分频计数器 → 各模块 tick_flag → 主循环消费:

| tick | 模块 | 周期 |
|------|------|------|
| gyro | 陀螺仪 | 1ms |
| encoder | 编码器 | 10ms |
| motion_control | 底层PID | 10ms |
| motion_manager | 运动规划 | 10ms |
| control_manager | 总控调度 | 10ms |
| display | 蓝牙仪表盘 | 5ms |
| cam/sensor | FIFO消费 | 无flag, 最快 |

---

## 构建与烧录

### 工具链

- **编译器**: `arm-none-eabi-gcc` (hard float ABI)
- **构建**: CMake 3.16+ + Ninja

### 配置

```bash
cmake --preset Debug
```

### 构建

```bash
cmake --build --preset Debug
```

产物: `build/Debug/VisualGuided_Car.elf` / `.bin` / `.hex`

开发模板：[stm32-vscode-template](https://github.com/mizuniuo01/stm32-vscode-template)

### 烧录

使用 ST-Link / J-Link 或串口 ISP 烧录 `build/Debug/VisualGuided_Car.bin` 到 `0x08000000`。

### 蓝牙调参

通过蓝牙小程序发送命令实时调整参数:

| 命令 | 功能 |
|------|------|
| `spd_kp_up/down` | 速度环 KP |
| `ang_kp_up/down` | 角度环 KP |
| `base_spd_up/down` | 基础速度 |
| `target_ang_up/down` | 角度目标 |
| `move_dist_up/down` | 分段距离 |
| `move_start` | 手动前进 |
| `rotate_start` | 手动旋转 |
| `rotate_right` | 右转 90° |
| `ctrl_toggle` | STOP/RUNNING 切换 |

---

## 目录结构

```
├── Core/              ← STM32CubeMX 生成 (HAL init, ISR, main)
├── Drivers/           ← CMSIS + STM32 HAL 库
├── Users/
│   ├── Inc/           ← 应用层头文件 (motor, pid, control_manager, ...)
│   └── Src/           ← 应用层实现
├── vision/            ← MaixCAM 视觉代码与模型
│   └── models/        ← YOLOv5 模型文件
├── cmake/             ← 工具链 cmake 文件
├── docs/              ← 协议、设计文档、任务说明
├── build/             ← 构建产物 (gitignore)
├── CMakeLists.txt
├── CMakePresets.json
├── VisualGuided_Car.ioc  ← CubeMX 工程文件
├── README.md
├── CHANGELOG.md
└── .gitignore
```

---

## 功能特性

| 功能 | 状态 | 说明 |
|------|:--:|------|
| 速度环 PID | ✓ | 双轮独立, 编码器反馈, count/10ms |
| 角度环 PID | ✓ | yaw unwrap, 绝对锁定, 微分-on-实际值 |
| 方向识别 | ✓ | YOLOv5: W/R/L + Y + STOP |
| 红绿灯 | ✓ | 绿灯检测 + debounce |
| 路口穿越 | ✓ | 灰度黑线检测 + 冷却 + 方向驱动 |
| 障碍物检测 | ✓ | 超声波, 100mm 阈值, 蜂鸣器报警 |
| 距离规划 | ✓ | encoder 积分 + PID 清除 + 角度锁定 |
| 旋转规划 | ✓ | gyro yaw 死区 ±3° |
| 方向补偿 | ✓ | 坐标系状态机, 自动修正转弯漂移 |
| 蓝牙仪表盘 | ✓ | 江协科技小程序, 12 行数据 |
| 蓝牙调参 | ✓ | 20 条指令, 实时调 PID/速度/角度 |
| 看门狗 | ✓ | IWDG, 主循环统一喂狗 |
| 错误处理 | ✓ | 三层架构: 传输/上报/处理 |

---

## 许可证

MIT License — 详见 [LICENSE](LICENSE) 文件。
