/**
 * @file    control_manager.c
 * @brief   顶层控制调度（状态机 + 任务集成）
 * @author  mizuniuo01
 * @date    2026-07-15
 * @version 1.0.0
 * @note    10ms tick，内部调用 perception → 状态机 → motion_manager → motion_control
 * @note    障碍物为最高优先级，检测到立即停车并响蜂鸣器
 * @note    路口动作由 junction_flag + direction 触发
 * @note    绿灯场景需等 all_black 后才执行路口动作（距离缩减 10cm）
 */

#include "control_manager.h"
#include "perception.h"
#include "motion_control.h"
#include "motion_manager.h"
#include "gyroscope.h"
#include "system.h"

volatile uint8_t control_manager_tick_flag = 0;

static control_normal_params_t normal_params = {
    .base_speed   = 30,
    .angle_enable = 1,
};
static control_plan_params_t plan_params = {
    .distance_mm = 100,
    .delta_deg   = 90.0f,
    .speed       = 30,
};

static control_manager_state_t state;
static control_run_substate_t  substate;

/* 路口逻辑 */
static uint8_t junction_pending;
static uint8_t saved_junction_direction;

/* 障碍物 */
static uint8_t obstacle_active;
static control_manager_state_t pre_obstacle_state;
static control_run_substate_t  pre_obstacle_substate;

/**
 * @brief  执行路口动作（发起距离移动规划）
 * @param  direction       路口方向（0=直行, 1=右转, 2=左转）
 * @param  green_scenario  是否为绿灯场景（距离缩减 10cm）
 * @retval 无
 */
static void execute_junction(uint8_t direction, uint8_t green_scenario)
{
    int16_t dist;

    dist = (direction == 0) ? JUNCTION_DIST_STRAIGHT_MM : JUNCTION_DIST_TURN_MM;
    if (green_scenario) {
        dist -= JUNCTION_DIST_GREEN_REDUCE;
        if (dist < 0) {
            dist = 0;
        }
    }

    motion_manager_start_move(dist, plan_params.speed);
    substate = CONTROL_RUN_JUNCTION_MOVE;
}

/**
 * @brief  control_manager 初始化
 * @param  无
 * @retval 无
 */
void control_manager_init(void)
{
    gyro_data_t gyro;

    state = CONTROL_MANAGER_STATE_STOP;
    substate = CONTROL_RUN_NORMAL;
    junction_pending = 0;
    saved_junction_direction = 0;
    obstacle_active = 0;

    gyro = gyro_get_data();
    *motion_control_get_target_angle_ptr() = gyro.yaw;
    motion_control_set_angle(gyro.yaw);
    motion_control_enable_angle(1);
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
    float *target_angle_ptr;
    float current_angle;

    if (!control_manager_tick_flag) {
        return;
    }
    control_manager_tick_flag = 0;

    /* 1. 感知数据采集 */
    perception_task();
    pd = perception_get_data();

    /* 2. 障碍物处理（最高优先级——上升沿停车+蜂鸣器，下降沿恢复） */
    if (pd->obstacle_flag) {
        if (!obstacle_active) {
            pre_obstacle_state    = state;
            pre_obstacle_substate = substate;
            motion_control_set_base_speed(0);
            motion_control_set_diff(0);
            motion_control_enable_angle(1);
            buzzer_on(system_buzzer());
            obstacle_active = 1;
        }
    } else {
        if (obstacle_active) {
            buzzer_off(system_buzzer());
            state    = pre_obstacle_state;
            substate = pre_obstacle_substate;
            obstacle_active = 0;
        }
    }

    /* 3. 状态机推进（障碍物激活时跳过） */
    if (!obstacle_active) {
        target_angle_ptr = motion_control_get_target_angle_ptr();
        current_angle    = *target_angle_ptr;

        switch (state) {
        case CONTROL_MANAGER_STATE_STOP:
            motion_manager_set_normal(0, current_angle, 0, 1);
            break;

        case CONTROL_MANAGER_STATE_RUNNING:
            switch (substate) {
            case CONTROL_RUN_NORMAL:
                /* 路口上升沿检测 */
                if (pd->junction_flag && !junction_pending) {
                    saved_junction_direction = pd->direction;
                    if (!pd->green) {
                        execute_junction(saved_junction_direction, 0);
                    } else {
                        junction_pending = 1;
                    }
                }

                /* all_black 触发绿灯场景的路口动作 */
                if (junction_pending && pd->all_black_flag) {
                    junction_pending = 0;
                    if (pd->green) {
                        execute_junction(saved_junction_direction, 1);
                    } else {
                        motion_manager_set_normal(0, current_angle, 0, 1);
                        substate = CONTROL_RUN_WAIT_GREEN;
                    }
                }

                /* 仍在 NORMAL：普通闭环（含感知差速） */
                if (substate == CONTROL_RUN_NORMAL) {
                    motion_manager_set_normal(normal_params.base_speed,
                        current_angle, pd->diff, normal_params.angle_enable);
                }
                break;

            case CONTROL_RUN_JUNCTION_MOVE:
                /* 等待移动完成 */
                if (motion_manager_get_state() == MOTION_MANAGER_STATE_NORMAL) {
                    if (saved_junction_direction == 0) {
                        substate = CONTROL_RUN_NORMAL;
                    } else {
                        float rot_ang = (saved_junction_direction == 1)
                            ? JUNCTION_ANGLE_RIGHT_DEG : JUNCTION_ANGLE_LEFT_DEG;
                        motion_manager_start_rotate(rot_ang, plan_params.speed);
                        substate = CONTROL_RUN_JUNCTION_ROTATE;
                    }
                }
                break;

            case CONTROL_RUN_JUNCTION_ROTATE:
                /* 等待旋转完成 */
                if (motion_manager_get_state() == MOTION_MANAGER_STATE_NORMAL) {
                    substate = CONTROL_RUN_NORMAL;
                }
                break;

            case CONTROL_RUN_WAIT_GREEN:
                /* 停车等待绿灯重新出现 */
                motion_manager_set_normal(0, current_angle, 0, 1);
                if (pd->green) {
                    execute_junction(saved_junction_direction, 1);
                }
                break;

            default:
                substate = CONTROL_RUN_NORMAL;
                break;
            }
            break;

        default:
            state = CONTROL_MANAGER_STATE_STOP;
            break;
        }
    }

    /* 4. 运动管理层推进（内部状态机消耗 tick_flag） */
    motion_manager_task();

    /* 5. 障碍物二次安全覆盖（防止 motion_manager 完成时恢复非零速度） */
    if (obstacle_active) {
        motion_control_set_base_speed(0);
        motion_control_set_diff(0);
    }

    /* 6. 底层闭环控制 */
    motion_control_task();
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
            state    = CONTROL_MANAGER_STATE_RUNNING;
            substate = CONTROL_RUN_NORMAL;
            junction_pending = 0;
        }
    } else {
        state    = CONTROL_MANAGER_STATE_STOP;
        substate = CONTROL_RUN_NORMAL;
        junction_pending = 0;
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
}
