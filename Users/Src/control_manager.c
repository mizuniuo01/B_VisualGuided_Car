/**
 * @file    control_manager.c
 * @brief   顶层控制调度（状态机 + 任务集成）
 * @author  mizuniuo01
 * @date    2026-07-16
 * @version 2.0.0
 * @note    10ms tick，内部调用 perception → 状态机 → motion_manager → motion_control
 * @note    控制方案：固定距离分段 (70cm) + 方向驱动循环
 * @note    障碍物为最高优先级，检测到立即停车并响蜂鸣器
 * @note    黑线 + 红绿灯：黑线处记录剩余距离，绿灯直行 / 红灯等待
 */

#include "control_manager.h"
#include "perception.h"
#include "motion_manager.h"
#include "system.h"
#include "buzzer.h"

/* TIM6 ISR 每 10ms 置位，control_manager_task 消费并清零 */
volatile uint8_t control_manager_tick_flag = 0;

/* 参数结构体（保留，bt_command 使用） */
static control_normal_params_t normal_params = {
    .base_speed = CONTROL_DEFAULT_SPEED,
    .angle_enable = 1,
};
static control_plan_params_t plan_params = {
    .distance_mm = CONTROL_SEGMENT_DISTANCE_MM,
    .delta_deg = CONTROL_DEFAULT_DELTA_DEG,
    .speed = CONTROL_DEFAULT_SPEED,
};

/* 顶层状态 */
static control_manager_state_t state;
static control_run_substate_t substate;

/* 方向追踪 */
static uint8_t last_direction; /* 最新有效方向（每 tick 更新） */

/* 黑线边缘检测 */
static uint8_t prev_all_black_flag;        /* 上一 tick 的黑线状态 */
static uint8_t black_line_cooldown_active; /* 黑线冷却激活 */
static int16_t black_line_cooldown_remaining; /* 冷却剩余距离（mm），独立累计，跨段有效 */
static int16_t cooldown_prev_elapsed; /* 上一 tick 的 elapsed_mm，用于计算步进 */

/* 启动延迟标志（障碍物恢复后下一 tick 启动运动） */
static uint8_t move_pending_start;
static uint8_t turn_pending_start;

/* 障碍物上下文保存 */
static uint8_t obstacle_active;
static control_run_substate_t pre_obstacle_substate;
static int16_t pre_obstacle_remaining_mm;
static uint8_t pre_obstacle_direction;

/* 黑线等待上下文 */
static int16_t saved_remaining_mm;

/* 手动指令标志（STOP 状态下跳过每 tick 速度清零） */
static uint8_t manual_override;

/* STOP 标志物请求（当前运动完成后停车） */
static uint8_t stop_requested;

static uint8_t direction_state;
static int16_t direction_counter[DIR_NUM];

/**
 * @brief  根据方向获取旋转角度
 * @param  direction  方向（1=右转, 2=左转）
 * @retval 旋转角度（度）
 */
static float get_turn_angle(uint8_t direction)
{
    if (direction == DIRECTION_RIGHT) {
        return CONTROL_TURN_ANGLE_RIGHT_DEG;
    }
    return CONTROL_TURN_ANGLE_LEFT_DEG;
}

/**
 * @brief  重置追踪状态变量（init 与 set_running 共用）
 * @param  无
 * @retval 无
 */
static void reset_tracking_state(void)
{
    uint8_t i;

    last_direction = 0;
    prev_all_black_flag = 0;
    black_line_cooldown_active = 0;
    black_line_cooldown_remaining = 0;
    cooldown_prev_elapsed = 0;
    move_pending_start = 0;
    turn_pending_start = 0;
    obstacle_active = 0;
    manual_override = 0;
    stop_requested = 0;
    direction_state = DIR_STATE_FRONT;
    for (i = 0; i < DIR_NUM; i++) {
        direction_counter[i] = 0;
    }
}

/**
 * @brief  control_manager 初始化
 * @param  无
 * @retval 无
 */
void control_manager_init(void)
{
    state = CONTROL_MANAGER_STATE_STOP;
    substate = CONTROL_RUN_MOVE;
    pre_obstacle_substate = CONTROL_RUN_MOVE;
    pre_obstacle_remaining_mm = 0;
    pre_obstacle_direction = 0;
    saved_remaining_mm = 0;

    reset_tracking_state();
    motion_manager_lock_angle();
}

/**
 * @brief  control_manager 主任务（10ms 周期）
 * @note   调用顺序：perception → 障碍物 → 状态机 → motion_manager → motion_control
 * @param  无
 * @retval 无
 */
void control_manager_task(void)
{
    perception_data_t *pd;

    if (!control_manager_tick_flag) {
        return;
    }
    control_manager_tick_flag = 0;

    /* 1. 感知数据采集 */
    perception_task();
    pd = perception_get_data();

    /* 2. 方向追踪（每 tick 更新，DIRECTION_INVALID 时保持上次值） */
    if (pd->direction != DIRECTION_INVALID) {
        last_direction = pd->direction;
    }

    /* 2.5 STOP 标志物检测 */
    if (pd->stop_flag) {
        stop_requested = 1;
    }

    /* 3. 障碍物处理（最高优先级——上升沿保存+停车，下降沿恢复） */
    if (pd->obstacle_flag) {
        if (!obstacle_active) {
            /* 保存上下文 */
            pre_obstacle_substate = substate;
            pre_obstacle_direction = last_direction;

            if (substate == CONTROL_RUN_MOVE) {
                pre_obstacle_remaining_mm = motion_manager_get_remaining_mm();
                if (pre_obstacle_remaining_mm <= 0) {
                    pre_obstacle_remaining_mm = plan_params.distance_mm;
                }
                motion_manager_cancel();
            } else if (substate == CONTROL_RUN_TURN) {
                motion_manager_cancel();
            }
            /* BLACK_LINE_WAIT：已停车 */

            motion_manager_hold_stop();
            buzzer_on(system_buzzer());
            obstacle_active = 1;
        }
    } else {
        if (obstacle_active) {
            buzzer_off(system_buzzer());

            /* 恢复上下文 */
            substate = pre_obstacle_substate;

            if (substate == CONTROL_RUN_MOVE) {
                if (pre_obstacle_remaining_mm > 0) {
                    saved_remaining_mm = pre_obstacle_remaining_mm;
                } else {
                    saved_remaining_mm = plan_params.distance_mm;
                }
                move_pending_start = 1;
            } else if (substate == CONTROL_RUN_TURN) {
                motion_manager_lock_angle();
                turn_pending_start = 1;
            }
            /* BLACK_LINE_WAIT：无需操作，继续等待绿灯 */

            obstacle_active = 0;
        }
    }

    /* 4. 状态机推进（障碍物激活时跳过） */
    if (!obstacle_active) {
        switch (state) {
            case CONTROL_MANAGER_STATE_STOP:
                if (manual_override) {
                    if (motion_manager_get_state() == MOTION_MANAGER_STATE_NORMAL) {
                        manual_override = 0;
                        motion_manager_halt();
                    }
                } else {
                    motion_manager_hold_stop();
                }
                break;

            case CONTROL_MANAGER_STATE_RUNNING:
                switch (substate) {
                    case CONTROL_RUN_MOVE:
                        /* 启动延迟处理（障碍物恢复后首次进入） */
                        if (move_pending_start) {
                            move_pending_start = 0;
                            motion_manager_start_move(saved_remaining_mm,
                                plan_params.speed);
                            break;
                        }

                        /* 黑线冷却进度更新（独立累计距离，跨段有效） */
                        if (black_line_cooldown_active) {
                            int16_t now = motion_manager_get_elapsed_mm();
                            int16_t step;
                            if (now < cooldown_prev_elapsed) {
                                /* move 被重启过，步进 = 当前段已走距离 */
                                step = now;
                            } else {
                                step = now - cooldown_prev_elapsed;
                            }
                            cooldown_prev_elapsed = now;
                            if (step > 0) {
                                if (black_line_cooldown_remaining > step) {
                                    black_line_cooldown_remaining -= step;
                                } else {
                                    black_line_cooldown_remaining = 0;
                                    black_line_cooldown_active = 0;
                                }
                            }
                        }

                        /* 黑线上升沿检测（冷却期内跳过） */
                        if (pd->all_black_flag && !prev_all_black_flag) {
                            prev_all_black_flag = 1;
                            if (!black_line_cooldown_active &&
                                motion_manager_get_state() == MOTION_MANAGER_STATE_MOVE) {
                                saved_remaining_mm = motion_manager_get_remaining_mm();
                                if (saved_remaining_mm > 0) {
                                    /* 激活冷却：记录剩余冷却距离 */
                                    black_line_cooldown_active = 1;
                                    black_line_cooldown_remaining =
                                        BLACK_LINE_COOLDOWN_MM;
                                    cooldown_prev_elapsed =
                                        motion_manager_get_elapsed_mm();

                                    if (pd->green) {
                                        /* 绿灯：无缝继续剩余距离 */
                                        motion_manager_replan_remaining_mm(
                                            saved_remaining_mm);
                                    } else {
                                        /* 无绿灯：停车等待 */
                                        motion_manager_cancel();
                                        motion_manager_halt();
                                        substate = CONTROL_RUN_BLACK_LINE_WAIT;
                                        break;
                                    }
                                }
                            }
                        }
                        if (!pd->all_black_flag) {
                            prev_all_black_flag = 0;
                        }

                        /* 距离规划完成检测 */
                        if (motion_manager_get_state() == MOTION_MANAGER_STATE_NORMAL) {
                            if (stop_requested) {
                                control_manager_set_running(0);
                                break;
                            }
                            if (last_direction == DIRECTION_RIGHT ||
                                last_direction == DIRECTION_LEFT) {
                                /* 右转 +1 / 左转 -1（环形） */
                                if (last_direction == DIRECTION_RIGHT) {
                                    direction_state = (direction_state + 1) % DIR_NUM;
                                } else {
                                    direction_state =
                                        (direction_state + DIR_NUM - 1) % DIR_NUM;
                                }
                                direction_counter[direction_state]++;

                                /* 发起旋转 */
                                motion_manager_start_rotate(
                                    get_turn_angle(last_direction), plan_params.speed);
                                substate = CONTROL_RUN_TURN;
                            } else {
                                /* 直行（direction=0 或无效默认直行） */
                                int16_t seg = plan_params.distance_mm;
                                seg -= direction_counter[direction_state] *
                                       CONTROL_COMPENSATION_UNIT_MM;
                                direction_counter[direction_state] = 0;
                                motion_manager_start_move(seg, plan_params.speed);
                                /* 保持在 MOVE */
                            }
                        }
                        break;

                    case CONTROL_RUN_TURN:
                        /* 启动延迟处理（障碍物恢复后首次进入） */
                        if (turn_pending_start) {
                            turn_pending_start = 0;
                            motion_manager_start_rotate(get_turn_angle(last_direction),
                                plan_params.speed);
                            break;
                        }

                        /* 旋转完成检测 */
                        if (motion_manager_get_state() == MOTION_MANAGER_STATE_NORMAL) {
                            if (stop_requested) {
                                control_manager_set_running(0);
                                break;
                            }
                            /* 旋转完成，计算补偿后前进距离 */
                            saved_remaining_mm = plan_params.distance_mm;
                            saved_remaining_mm -= direction_counter[direction_state] *
                                                  CONTROL_COMPENSATION_UNIT_MM;
                            direction_counter[direction_state] = 0;
                            move_pending_start = 1;
                            substate = CONTROL_RUN_MOVE;
                        }
                        break;

                    case CONTROL_RUN_BLACK_LINE_WAIT:
                        /* 确保停车 */
                        motion_manager_hold_stop();

                        /* 等待绿灯 */
                        if (pd->green) {
                            motion_manager_start_move(saved_remaining_mm,
                                plan_params.speed);
                            substate = CONTROL_RUN_MOVE;
                        }
                        break;

                    default:
                        substate = CONTROL_RUN_MOVE;
                        break;
                }
                break;

            default:
                state = CONTROL_MANAGER_STATE_STOP;
                break;
        }
    }

    /* 5. 运动管理层推进 */
    motion_manager_task();

    /* 6. 障碍物二次安全覆盖（防止 motion_manager 恢复非零速度） */
    if (obstacle_active) {
        motion_manager_halt();
    }

    /* 7. 底层闭环控制 */
    motion_manager_run_control_task();
}

/**
 * @brief  设置运行状态
 * @param  run  0=停止, 非零=运行
 * @retval 无
 */
void control_manager_set_running(uint8_t run)
{
    if (run) {
        if (state == CONTROL_MANAGER_STATE_STOP) {
            state = CONTROL_MANAGER_STATE_RUNNING;
            substate = CONTROL_RUN_MOVE;
            saved_remaining_mm = plan_params.distance_mm;

            reset_tracking_state();
            motion_manager_lock_angle();

            motion_manager_start_move(plan_params.distance_mm, plan_params.speed);
        }
    } else {
        state = CONTROL_MANAGER_STATE_STOP;
        substate = CONTROL_RUN_MOVE;
        motion_manager_cancel();
        motion_manager_halt();
    }
}

control_manager_state_t control_manager_get_state(void)
{
    return state;
}

control_run_substate_t control_manager_get_substate(void)
{
    return substate;
}

const control_normal_params_t *control_manager_get_normal_params(void)
{
    return &normal_params;
}

void control_manager_set_normal_params(const control_normal_params_t *p)
{
    if (!p) {
        return;
    }
    normal_params = *p;
    /* 同步速度到运动规划参数 */
    plan_params.speed = p->base_speed;
}

const control_plan_params_t *control_manager_get_plan_params(void)
{
    return &plan_params;
}

void control_manager_set_plan_params(const control_plan_params_t *p)
{
    if (!p) {
        return;
    }
    plan_params = *p;
    /* 同步速度到基础参数 */
    normal_params.base_speed = p->speed;
}

/**
 * @brief  设置手动指令模式（STOP 状态下允许运动）
 * @note   蓝牙手动指令（move_start/rotate_start/rotate_right）调用前置位，
 *         运动完成后自动清零
 * @param  enable  非零启用手动模式，0 关闭
 * @retval 无
 */
void control_manager_set_manual_override(uint8_t enable)
{
    manual_override = enable ? 1 : 0;
}
