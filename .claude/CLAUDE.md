# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于 STM32F407XX 的视觉引导小车（26 电赛训练赛 B 题），使用 STM32CubeMX 生成 HAL 代码，CMake + Ninja 构建。

**当前阶段**：v1.0.0 完成，所有模块已实现并通过测试。

## 构建命令

```bash
cmake --preset Debug
cmake --build --preset Debug
```

工具链：`arm-none-eabi-gcc`（需在 PATH 中）。目标：Cortex-M4 + FPU hard float。

## 架构分层

```
vision/           ← MaixCAM 视觉代码 + YOLOv5 模型（Python）
Users/            ← 应用层（驱动 + 控制）
Core/             ← STM32CubeMX 生成（HAL init、ISR、main）
Drivers/          ← CMSIS + STM32 HAL 库
```

- **`Users/Inc` / `Users/Src`**：所有应用模块。模块间通过 `system.h/c` 的 getter 获取句柄，不直接 `extern`。
- **`Core/Src/main.c`**：平台初始化 + 主循环（见下方）+ HAL 回调集中处理。
- **控制分层**：`control_manager` → `perception` → `motion_manager` → `motion_control` → `driver`

### 主循环任务顺序

```
gyro_task → ultrasonic_task → cam_task → sensor_task
  → blueteeth_task → display_task → control_manager_task
```

`control_manager_task` 内部串联：perception → 状态机 → motion_manager → motion_control。

## 硬件资源分配

| 外设 | 引脚 | 用途 | 模式 |
|------|------|------|------|
| TIM1 | PE9 CH1, PE11 CH2 | 左编码器 | QEI |
| TIM2 | PA0 CH1, PA1 CH2 | 右编码器 | QEI |
| TIM3 | PA6 CH1, PA7 CH2 | 电机 PWM | PWM, ARR=8400 |
| TIM4 | PD15 CH4 | 超声波 | 输入捕获 |
| TIM6 | — | 系统 tick 1ms | 基础定时器 |
| USART1 | PA9 TX, PA10 RX | 蓝牙 | DMA+IDLE |
| USART3 | PB10 TX, PB11 RX | MaixCAM | DMA+IDLE |
| USART6 | PC6 TX, PC7 RX | 陀螺仪 | DMA+IDLE |
| I2C2 | PB10 SCL, PB11 SDA | 灰度传感器 | DMA, 100kHz |
| I2C3 | PA8 SCL, PC9 SDA | OLED | 100kHz |
| GPIO | PA4/PA5 | 左电机 nSLEEP/PH + 蜂鸣器 | 推挽 |
| GPIO | PC4/PC5 | 右电机 PH/nSLEEP | 推挽 |
| IWDG | — | 看门狗 | 主循环刷新 |

## 模块职责

| 模块 | 职责 |
|------|------|
| `motor` | DRV8874 双路电机，PH/EN PWM 控制 |
| `encoder` | QEI 编码器，13PPR × 4 × 1:28 |
| `cam` | MaixCAM USART3 DMA+IDLE, 5 字节帧协议 |
| `gyroscope` | JY901S 姿态传感器 UART6 DMA |
| `blueteeth` | 蓝牙 USART1 DMA+IDLE 双向 FIFO |
| `ultrasonic` | 超声波 TIM4 输入捕获 |
| `sensor` | 八路灰度 I2C2 DMA, 0x4C |
| `display` | 蓝牙仪表盘 12 行数据输出 |
| `error_handler` | 三层错误管理（传输/上报/处理） |
| `buzzer`/`led`/`pwm` | 基础外设控制 |
| `system` | 硬件句柄注册中心 |
| `pid` | PID 控制器（微分-on-实际值） |
| `motion_control` | 角度环 + 双轮速度环, 10ms, yaw unwrap |
| `motion_manager` | 运动控制入口：普通闭环 + 距离/角度规划。对外提供 `halt`/`hold_stop`/`lock_angle` 透传，`control_manager` 禁止直接调 `motion_control` |
| `control_manager` | 顶层状态机：STOP/RUNNING + MOVE/TURN/BLACK_LINE_WAIT 子状态。方向坐标系补偿（0-3 环形）、黑线冷却、障碍物优先、STOP 标志 |
| `perception` | 视觉 + 灰度 + 超声波数据融合。direction 过滤（3→保持旧值）、黑线/障碍物/STOP 标志位 |

## 关键设计决策

- **分段循环**：固定 685mm 段 × direction 驱动，段末检测方向 → 直行继续 / 转弯
- **方向补偿**：坐标系 0-3 环形状态机，右转 +1 / 左转 -1，每方向独立计数器。当前方向有累积时下一段减去 `counter × 100mm`。自动修正转弯漂移
- **黑线冷却**：独立距离累计 500mm，跨段有效，过滤十字路口出口侧黑线
- **障碍物**：上升沿保存上下文 → emergency stop + 蜂鸣器 → 下降沿恢复
- **STOP 标志**：YOLO 检测 → `stop_requested` → 当前段完成后 → STOP
- **手动指令**：`manual_override` 标志位，STOP 状态下跳过每 tick 速度清零

## 核心设计约定

以下来自 `docs/CODING_STANDARD.md`，严格遵守 BARR-C:2018：

- **命名**：函数/变量 `snake_case`，类型 `snake_case_t`，宏 `UPPER_SNAKE_CASE`
- **非阻塞**：严禁 `HAL_Delay()`，统一 tick 调度（TIM6 ISR 置 flag，主循环消费）
- **状态机**：有状态模块用 `typedef enum` + `switch` 推进
- **UART 通信**：DMA + IDLE + 环形 FIFO
- **句柄模式**：默认多实例，内部状态 `static`，通过 `system.h/c` getter 共享
- **禁止项**：`malloc`/`free`、递归、VLA、ISR 中 `printf`、`goto` 向前跳转、条件中赋值
- **注释**：中文。`.c` 有 `@brief/@param/@retval`，`.h` 无函数文档注释
- **格式**：K&R 花括号，4 空格缩进，90 列宽
- **宏/enum**：≤3 个相关值用 `#define`，>3 个用 `typedef enum`

## CubeMX 代码修改

CubeMX 重新生成会覆盖 `Core/`（`USER CODE BEGIN/END` 区域保留）。用户代码放 `Users/`。

## 添加新模块

1. `Users/Inc/` + `Users/Src/` 创建 `.h/.c`
2. 顶层 `CMakeLists.txt` 的 `target_sources` 添加 `.c`
3. 共享句柄：`system.h` 声明 getter → `system.c` 定义
