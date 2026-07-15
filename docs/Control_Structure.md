```markdown
# 嵌入式运动控制软件架构说明

> 注意：本文档仅作为软件架构设计参考，用于帮助理解模块职责和整体关系。实际代码实现不要求 100% 完全按照本文档执行，应以用户后续明确指示和实际工程需求为准。


## 总体架构

系统采用分层设计：

1. control_manager（总控制调度层）
2. perception（感知层）
3. motion_manager（运动管理层）
4. motion_control（底层闭环控制层）
5. driver（硬件驱动层）


整体关系：

```

```
             control_manager
                    |
                    |
              perception
                    |
                    |
             motion_manager
                    |
          ---------------------
          |                   |
   direct control        motion planning
          |                   |
          ---------------------
                    |
             motion_control
                    |
          -----------------
          |               |
      angle loop      speed loop
                    |
              motor driver
```

```


---

# 1. control_manager

文件：

```

control_manager.c

```

职责：

系统最高层控制调度。

负责：

- 整体任务逻辑
- 状态机管理
- 工作模式切换
- 综合感知信息
- 决定当前执行动作
- 调用 motion_manager 执行运动


示例：

```

获取 perception 信息

↓

判断当前状态

↓

调用 motion_manager 执行动作

```


不负责：

- PID计算
- 电机控制
- 底层硬件访问
- 具体闭环实现


---

# 2. perception

文件：

```

perception.c

```

职责：

环境感知和传感器数据处理。


包含：

- 视觉数据处理
- 灰度传感器数据处理
- 特殊线检测
- 环境状态判断
- 其他传感器信息融合


注意：

灰度传感器不是用于传统巡线。

用途：

```

灰度传感器

↓

特殊线检测

↓

向上层提供状态信息

```


perception 输出：

```

环境信息

```

而不是：

```

控制指令

```


---

# 3. motion_manager

文件：

```

motion_manager.c

```

职责：

运动控制统一管理入口。


负责：

- 普通运动控制
- 运动规划逻辑
- 控制模式管理
- 运动目标生成
- 调用底层闭环控制


motion_manager 内部包含两种主要控制方式。


---

## 方式1：普通闭环控制

直接指定：

- 目标速度
- 目标角度


流程：

```

目标速度 / 目标角度

↓

motion_control

```


适用于：

- 手动控制
- 简单运动
- 直接控制需求


---

## 方式2：运动规划控制

指定：

- 移动距离
- 旋转角度


motion_manager 内部完成：

```

目标距离 / 目标角度

↓

计算当前运动状态

↓

生成目标速度 / 目标角度

↓

motion_control

```


例如：

```

Move_Distance(500mm)

Rotate_Angle(90deg)

```


---

## motion_manager 控制权管理

运动控制只能存在一个控制来源。


错误：

```

direct control
|
|
motion planning
|
|
同时控制 motion_control

```


正确：

```

motion_manager

选择当前控制模式

↓

生成唯一运动目标

↓

motion_control

```


motion_manager 不直接实现：

- PID算法
- PWM输出
- 电机驱动


---

# 4. motion_control

文件：

```

motion_control.c

```


职责：

底层闭环控制。


负责：

- 角度环
- 速度环


结构：

```

目标角度

↓

角度环 PID

↓

目标速度

↓

速度环 PID

↓

PWM输出

↓

电机

```


motion_control 不关心：

- 为什么需要这个角度
- 为什么移动这个距离
- 当前任务是什么
- 上层状态是什么


它只负责：

```

稳定跟踪目标

```


接口示例：

```

MotionControl_SetAngle()

MotionControl_SetSpeed()

MotionControl_Update()

```


---

# 5. Driver

目录：

```

driver/

```


负责硬件访问：

例如：

```

motor_driver.c
encoder_driver.c
imu_driver.c
gray_driver.c
camera_driver.c

```


职责：

- 操作具体硬件
- 提供底层接口


不包含：

- 控制策略
- 状态机
- 运动规划
- 任务逻辑


---

# 最终目录结构

```

Application/

control/
├── control_manager.c
├── perception.c
└── motion_manager.c

control/
└── motion_control.c

driver/
├── motor_driver.c
├── encoder_driver.c
├── imu_driver.c
├── gray_driver.c
└── camera_driver.c

```


---

# 核心设计原则

## 1. motion_manager 是运动控制入口

所有运动请求：

- 速度控制
- 角度控制
- 移动距离
- 旋转角度

统一通过 motion_manager。


---

## 2. motion_control 只负责闭环执行

不要在 motion_control 中加入：

- 距离规划
- 动作逻辑
- 状态判断
- 任务决策


---

## 3. 规划逻辑属于 motion_manager

不存在独立：

```

motion_planner.c

```


---

# 核心思想

```

control_manager
负责决定做什么

perception
负责感知环境

motion_manager
负责决定如何产生运动目标

motion_control
负责稳定执行运动目标

driver
负责操作硬件

```
```

---

# motion_control 层实现方案（2026-07-15）

底层闭环层第一版实现方案。范围仅覆盖 motion_control 自身，上层
motion_manager 与 control_manager 后续单独讨论。

## 反馈来源

- **速度反馈**：`encoder_get_left()` / `encoder_get_right()`，单位为
  count/10ms。数值尺度与占空比对应（100 counts ≈ 100% 占空比 ≈ PWM 8400）。
  左右轮各自读取自己的编码器，独立闭环。
- **角度反馈**：`gyro_get_data().yaw`，直接取最新值不做平均或滤波。陀螺仪
  以 1ms 更新，角度环以 10ms 消费，取最新 yaw 是相位最优选择。若日后
  出现明显抖动，考虑加一阶低通而非滑动平均。

## 三通道输入

motion_control 提供三个 setter 作为运行时输入：

| 接口 | 语义 | 单位 |
|------|------|------|
| `motion_control_set_base_speed(int16_t)` | 左右轮共用基础速度 | count/10ms |
| `motion_control_set_angle(float)` | 目标绝对角度（yaw 锁定） | 度 |
| `motion_control_set_diff(int16_t)` | 外部差速（视觉巡线等叠加项） | count/10ms |

角度环使用**绝对角度锁定**，累积转角语义在 motion_manager 层实现。

## 数据流（10ms 周期）

```
1. 读编码器：enc_l = encoder_get_left(), enc_r = encoder_get_right()
2. 读陀螺仪：yaw_now = gyro_get_data().yaw
3. 若角度环使能：
     angle_diff = pid_calc(angle_pid, target_angle, yaw_now)
   否则 angle_diff = 0
4. total_diff  = angle_diff + external_diff
5. left_target  = base_speed - total_diff
   right_target = base_speed + total_diff
6. left_pwm  = pid_calc(speed_pid_l, left_target,  enc_l)
   right_pwm = pid_calc(speed_pid_r, right_target, enc_r)
7. motor_set_speed_left(&motor, htim3, left_pwm)
   motor_set_speed_right(&motor, htim3, right_pwm)
```

## 角度环独立开关

`motion_control_enable_angle(uint8_t enable)` 控制 angle_diff 是否进入计算，
方便单独调试速度环和角度环。关闭时 angle_diff 强制为 0，PID 状态清零。

## PID 实例与调参

motion_control 内部持有 3 个 pid_t 实例：
- 左轮速度 PID
- 右轮速度 PID
- 角度环 PID

左右轮速度 PID **共享参数**（同批电机，独立整定意义不大且易失衡）。

3 个 PID 句柄统一注册到 system.c/.h，通过 getter 暴露给上层调参：

```c
pid_t *system_get_speed_pid_left(void);
pid_t *system_get_speed_pid_right(void);
pid_t *system_get_angle_pid(void);
```

上层调参使用 pid.h 提供的通用接口：

```c
pid_param_t p = { .kp=..., .ki=..., .kd=..., .out_max=..., .integral_max=... };
pid_set_param(system_get_speed_pid_left(),  &p);
pid_set_param(system_get_speed_pid_right(), &p);
```

## pid.h 扩展

新增参数结构体与句柄级 setter/getter，`pid_init` 内部改用 `pid_set_param`
实现：

```c
typedef struct {
    float kp;
    float ki;
    float kd;
    float out_max;
    float integral_max;
} pid_param_t;

void pid_set_param(pid_t *pid, const pid_param_t *param);
void pid_get_param(const pid_t *pid, pid_param_t *param);
```

## Task 调度

- 单一 task：`motion_control_task()`，10ms 周期
- tick 标志：`volatile uint8_t motion_control_tick_flag`，由 TIM6 ISR 每
  10ms 置位（复用现有 encoder_tick_cnt 相同的分频逻辑或独立计数器）
- 主循环消费 flag，执行完整数据流

## 停车 / 使能

motion_control 本层**不提供** start/stop 接口。这是上层 motion_manager 的
运动规划模式与普通闭环模式的切换语义，不应污染底层。上层若要停车，直接
`set_base_speed(0)` + `enable_angle(0)` 即可。

## 接口清单

```c
/* 初始化 */
void motion_control_init(void);

/* 主任务 */
void motion_control_task(void);
extern volatile uint8_t motion_control_tick_flag;

/* 运动输入 */
void motion_control_set_base_speed(int16_t base_speed);
void motion_control_set_angle(float target_angle_deg);
void motion_control_set_diff(int16_t external_diff);

/* 角度环开关 */
void motion_control_enable_angle(uint8_t enable);
```

PID 调参统一走 system.c getter + pid.h 通用接口，motion_control 不再
重复封装 set_pid/get_pid。

## 测试记录（2026-07-15）

motion_control 模块硬件测试全部通过：
- 速度环：`set_base_speed()` 双轮独立闭环正常，编码器反馈稳定
- 角度环：`set_angle()` 绝对角度锁定正常，yaw unwrap 跨 ±180° 无突变
- 角度环开关：`enable_angle(0)` 完全禁用 angle_diff，速度环不受影响
- 外部差速：`set_diff()` 叠加正常，与 angle_diff 互不冲突
- 10ms 周期 tick 调度正常，task 内完整数据流无阻塞
- PID 调参路径：system_pid_xxx() → pid_set_param/get_param 可用

**状态：motion_control 层完成，进入 motion_manager 设计。**
