#ifndef MOTION_MANAGER_H
#define MOTION_MANAGER_H

#include <stdint.h>

/* 运动管理层状态 */
typedef enum {
    MOTION_MANAGER_STATE_NORMAL = 0, /* 普通闭环 */
    MOTION_MANAGER_STATE_MOVE,       /* 运动规划-距离 */
    MOTION_MANAGER_STATE_ROTATE,     /* 运动规划-角度 */
} motion_manager_state_t;

extern volatile uint8_t motion_manager_tick_flag;

void motion_manager_init(void);
void motion_manager_task(void);

/* 普通闭环 —— 统一接口，内部分发给 motion_control */
void motion_manager_set_normal(int16_t base_speed, float target_angle_deg,
    int16_t external_diff, uint8_t angle_enable);

/* 运动规划 —— 异步发起，task 内推进 */
void motion_manager_start_move(int16_t distance_mm, int16_t speed);
void motion_manager_start_rotate(float delta_deg, int16_t speed);

/* 查询当前状态 */
motion_manager_state_t motion_manager_get_state(void);

/* 查询运动规划目标参数（指针输出） */
void motion_manager_get_plan_params(int16_t *distance_mm, float *delta_deg);

#endif
