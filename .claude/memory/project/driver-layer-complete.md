---
name: driver-layer-complete
description: 所有基础驱动模块已开发完成并通过硬件测试，进入控制算法阶段
metadata:
  type: project
---

所有基础驱动层模块已开发并测试完毕。Why: 2026-07-14 完成 encoder/motor 模块提交 (7486692)。How to apply: 后续工作聚焦于 PID 速度闭环、巡线算法、摄像头视觉引导等上层控制逻辑，不应再修改驱动层的基本通信协议。

**已完成的模块**：
- blueteeth: 蓝牙 DMA 双向通信，@...# 帧协议，江协科技小程序显示
- gyroscope: 姿态传感器 (UART6 DMA)，roll/pitch/yaw
- ultrasonic: 超声波测距 (TIM4 CH4 输入捕获)
- sensor: 八路灰度传感器 (I2C2 DMA, 0x4C, 0xDD命令)
- cam: MaixCAM 视觉模块 (USART3 DMA, 0xFF/0xFE帧协议, is_junction/direction/green)
- motor: DRV8874 双路电机驱动 (TIM3 CH1/CH2 PWM, PA4/PC5 nSLEEP, PA5/PC4 PH)
- encoder: QEI 编码器 (TIM1 PE9/PE11 左, TIM2 PA0/PA1 右, 13PPR 四倍频 52counts/rev, 1:28减速)
- pwm: PWM 比较值薄封装
- led/buzzer: GPIO 控制外设
- error_handler: 统一错误管理
- display: 蓝牙仪表盘数据汇总显示
- system: 硬件句柄注册中心

**已知问题**：
- 编码器 13PPR 分辨率极低，1ms 扫描满速仅 ~9 counts，建议 10ms 累计或换高 PPR 编码器
- buzzer 与左电机方向共用 PA5（硬件设计决定）
- 左编码器值取反 (`encoder_left_val = -diff_value`) 适配安装方向
