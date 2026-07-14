# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于 STM32F407XX 的视觉引导小车，使用 STM32CubeMX 生成 HAL 代码，CMake + Ninja 构建。

**当前阶段**：驱动层全部完成，即将进入 PID 速度闭环和巡线算法阶段。

## 构建命令

```bash
# 配置（Debug）
cmake --preset Debug

# 构建
cmake --build --preset Debug

# 或手动指定
cmake -B build/Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug
```

工具链：`arm-none-eabi-gcc`（需在 PATH 中）。目标：Cortex-M4 + FPU hard float。

## 架构分层

```
Users/           ← 应用层驱动模块（motor, encoder, pid, cam, gyroscope, blueteeth 等）
Core/            ← STM32CubeMX 生成的 HAL 初始化代码（main.c, gpio.c, stm32f4xx_it.c 等）
Drivers/CMSIS/   ← ARM CMSIS
Drivers/STM32F4xx_HAL_Driver/ ← STM32 HAL 库
cmake/            ← 工具链文件和 CubeMX 构建集成
```

- **`Users/Inc` / `Users/Src`**：所有应用模块。每个模块有 `.h`（接口）和 `.c`（实现）。模块间通过 `system.h/c` 的 getter 函数获取句柄，不直接 `extern` 全局变量。
- **`Core/Src/main.c`**：平台初始化（HAL_Init、时钟、GPIO）+ 主循环 + HAL 回调集中处理。
- **中断服务函数** 写在 `Core/Src/stm32f4xx_it.c`，回调实现在对应 `Users/` 模块中。

## 硬件资源分配

| 外设 | 引脚 | 用途 | 模式 |
|------|------|------|------|
| TIM1 | PE9 (CH1), PE11 (CH2) | 左编码器 | QEI 四倍频 |
| TIM2 | PA0 (CH1), PA1 (CH2) | 右编码器 | QEI 四倍频 |
| TIM3 | PA6 (CH1), PA7 (CH2) | 电机 PWM | PWM, ARR=8400 |
| TIM4 | PD15 (CH4) | 超声波测距 | 输入捕获 |
| TIM6 | — | 系统 tick 1ms | 基础定时器 |
| USART1 | PA9 (TX), PA10 (RX) | 蓝牙通信 | DMA+IDLE |
| USART3 | PB10 (TX), PB11 (RX) | 视觉模块 MaixCAM | DMA+IDLE |
| USART6 | PC6 (TX), PC7 (RX) | 陀螺仪 | DMA+IDLE |
| I2C2 | PB10 (SCL), PB11 (SDA) | 八路灰度传感器 | DMA, 100kHz |
| I2C3 | PA8 (SCL), PC9 (SDA) | OLED 显示屏 | 100kHz |
| GPIO | PA4 | 左电机 nSLEEP | 推挽输出 |
| GPIO | PA5 | 左电机 PH + 蜂鸣器 | 推挽输出 |
| GPIO | PC4 | 右电机 PH | 推挽输出 |
| GPIO | PC5 | 右电机 nSLEEP | 推挽输出 |
| IWDG | — | 看门狗 | 主循环刷新 |

## 模块职责

| 模块 | 职责 | 状态 |
|------|------|:---:|
| `motor` | DRV8874 双路直流有刷电机驱动，PH/EN 模式 PWM 控制（TIM3 CH1/CH2） | ✓ |
| `encoder` | QEI 编码器（TIM1 左 / TIM2 右），13PPR 四倍频 52 counts/rev，1:28 减速 | ✓ |
| `pwm` | PWM 比较值薄封装（`__HAL_TIM_SET_COMPARE`） | ✓ |
| `cam` | MaixCAM 视觉模块，USART3 DMA+IDLE+环形 FIFO，0xFF/0xFE 帧协议 | ✓ |
| `gyroscope` | 姿态传感器 UART6 DMA，输出 roll/pitch/yaw | ✓ |
| `blueteeth` | 蓝牙串口 USART1，`@...#` 帧协议，双向 DMA+FIFO | ✓ |
| `ultrasonic` | 超声波测距，TIM4 CH4 输入捕获 | ✓ |
| `sensor` | 感为科技八路灰度传感器，I2C2 DMA，地址 0x4C，命令 0xDD | ✓ |
| `display` | 蓝牙仪表盘，汇总各模块数据通过 `blueteeth_display()` 输出到手机 | ✓ |
| `error_handler` | 集中错误管理：传输→上报→处理 三层架构 | ✓ |
| `buzzer` / `led` | 基础 GPIO 控制外设 | ✓ |
| `system` | 硬件句柄注册中心，所有共享句柄的唯一定义处 | ✓ |
| `pid` | PID 控制器（速度闭环） | 待实现 |

## 核心设计约定

以下来自 `docs/CODING_STANDARD.md`，严格遵守 BARR-C:2018：

- **命名**：函数/变量 `snake_case`，类型 `snake_case_t`，宏 `UPPER_SNAKE_CASE`。所有公开标识符以模块名为前缀（如 `motor_set_speed`）。
- **非阻塞**：严禁 `HAL_Delay()` 或死循环等标志位。统一 tick 调度：硬件定时器 ISR 置 `volatile uint8_t xxx_tick_flag`，主循环调 `xxx_task()`。
- **状态机**：有状态的外设用 `typedef enum` 定义状态，在 `xxx_task()` 中 `switch` 推进。
- **UART 通信**：DMA + IDLE 中断 + 环形 FIFO。ISR 写 FIFO，task 消费。
- **句柄模式**：默认多实例设计，句柄由调用者分配。模块内部状态 `static`。
- **错误处理**：驱动出错调 `error_report(source, code)`，error_handler 统一上报和恢复。
- **禁止项**：`malloc`/`free`、递归、VLA、`printf` 在 ISR 中、`goto` 向前跳转、条件中赋值。
- **注释**：中文。`.c` 文件头 + 每个函数 `@brief/@param/@retval`。`.h` 文件不写函数文档注释。
- **花括号**：控制流 K&R 风格，函数 Allman 风格。
- **缩进**：4 空格，列宽 90。

详细规范见 `docs/CODING_STANDARD.md`，开发流程见 `docs/Development_Workflow.md`。

## 修改 CubeMX 生成的代码

CubeMX 重新生成会覆盖 `Core/` 下的内容（带 `USER CODE BEGIN/END` 标记的区域保留）。用户代码应放在 `Users/` 目录下，通过 `CMakeLists.txt` 顶层 `target_sources` 添加。

## 添加新模块

1. 在 `Users/Inc/` 和 `Users/Src/` 创建 `.h`/`.c`
2. 在顶层 `CMakeLists.txt` 的 `target_sources` 中添加 `.c` 文件
3. 如果模块有共享句柄：在 `system.h` 声明 getter，在 `system.c` 定义句柄和 getter
