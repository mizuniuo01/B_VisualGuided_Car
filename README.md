# 26电赛训练赛B题视觉引导小车

## MCU/模块
使用**stm32f407vet6**作为主控芯片

使用模块有：
- LED
- 蜂鸣器
- JY901S姿态传感器
- 轮趣科技编码电机
- 正点原子蓝牙模块
- MaixCam

## 环境
编译链、工具链、开发流使用我自主配置的stm32开发模板
https://github.com/mizuniuo01/stm32-vscode-template

## 项目目标
完成docs/B题.pdf内的题目

主要功能：
- 串级pid实现运动规划
- Yolov5实现方向标志物识别与红绿灯识别
- 灰度传感器模块与超声波模块实现黑线停止与障碍物检测