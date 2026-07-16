# Changelog

## [v1.0.0] - 2026-07-16

### 硬件驱动层
- motor: DRV8874 双路直流有刷电机, PH/EN PWM 控制
- encoder: QEI 编码器 TIM1/TIM2, 13PPR × 4 倍频 × 1:28 减速
- gyroscope: JY901S 姿态传感器 UART6 DMA+IDLE, roll/pitch/yaw
- ultrasonic: HC-SR04 超声波 TIM4 输入捕获
- sensor: 感为科技八路灰度 I2C2 DMA, 0x4C 地址
- cam: MaixCAM 视觉模块 USART3 DMA+IDLE, 5 字节帧协议
- blueteeth: 蓝牙 USART1 DMA+IDLE 双向 FIFO, 江协小程序协议
- display: 蓝牙仪表盘 12 行数据实时刷新
- buzzer/led: 基础 GPIO 外设
- system: 硬件句柄注册中心, getter 模式
- error_handler: 三层集中错误管理 (传输/上报/处理)
- iwdg: 独立看门狗, 主循环统一喂狗

### 底层闭环
- motion_control: 角度环 + 双轮速度环, 10ms tick
- PID: 微分-on-实际值, yaw unwrap + target align 跨 ±180°
- 三通道输入: base_speed / target_angle / external_diff

### 运动规划
- motion_manager: 普通闭环透传 + 距离/角度运动规划
- 距离规划: encoder 积分, 角度锁定护航
- 旋转规划: gyro yaw 死区 ±3°
- 规划辅助: elapsed/remaining/cancel/replan

### 环境感知
- perception: 视觉 + 灰度 + 超声波数据融合
- 方向追踪: 0/1/2/3 过滤, 3 无效保持上次
- 黑线检测: pattern_lookup PATTERN_CROSS
- 障碍物: 超声波 100mm 阈值
- STOP 标志: YOLO detection → 当前段完成后停车

### 顶层控制
- control_manager: 完整状态机 STOP/RUNNING + MOVE/TURN/BLACK_LINE_WAIT 子状态
- 分段方向驱动: 固定距离 685mm 循环, direction 决定直行/转弯
- 黑线逻辑: 上升沿检测 → 剩余距离记录 → 绿灯穿越 / 红灯等待
- 黑线冷却: 独立累计距离 500mm, 跨段有效, 过滤出口黑线
- 障碍物: 最高优先级, 上升沿紧急停车 + 蜂鸣器, 下降沿恢复
- 方向坐标系补偿: 前后左右 0-3 环形状态机, 四路计数器自动修正转弯漂移
- 手动指令: manual_override 机制, 蓝牙 move/rotate 命令

### 视觉 (MaixCAM)
- YOLOv5 推理: W/R/L 方向标志 + Y 绿灯 + STOP 停车标志
- 独立 debounce: direction/green 与 stop 各自防抖
- center contour scan 巡线 + junction detector 路口检测
- 5 字节帧协议: [is_junction, direction, green, stop, deviation]
- byte stuffing 转义: 0xFF/0xFE/0x7D

### 蓝牙调参
- 22 条指令: 速度/角度 PID 6 项, base_speed/target_angle 调参, move/rotate 手动控制
- ctrl_toggle: STOP/RUNNING 状态切换
- ff_up/down: 视觉前馈系数调节

### 工程化
- CMake + Ninja: arm-none-eabi-gcc, Cortex-M4 + FPU hard float
- BARR-C:2018 编码规范: snake_case, 4 空格, 90 列, enum+define 魔法数字管理
- 分层架构: control_manager → perception → motion_manager → motion_control → driver
- 薄透传: motion_manager 统一封装 motion_control 调用
- TIM6 1ms ISR 分频调度, 非阻塞状态机设计
