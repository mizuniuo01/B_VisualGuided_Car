/**
 * @file    control_manager.c
 * @brief   顶层控制调度（临时代码：默认普通闭环，蓝牙指令触发运动规划）
 * @author  mizuniuo01
 * @date    2026-07-15
 */

#include "control_manager.h"
#include "motion_control.h"
#include "motion_manager.h"
#include "gyroscope.h"

volatile uint8_t control_manager_tick_flag = 0;

static int16_t base_speed    = 10;
static int16_t diff          = 0;
static uint8_t angle_enable  = 1;
static int16_t plan_distance = 500;
static float  plan_delta     = 90.0f;
static int16_t plan_speed    = 30;

int16_t cm_get_base_speed(void)          { return base_speed; }
void    cm_set_base_speed(int16_t s)     { base_speed = s; motion_control_set_base_speed(s); }
float   cm_get_target_angle(void)        { return *motion_control_get_target_angle_ptr(); }
void    cm_set_target_angle(float a)     { *motion_control_get_target_angle_ptr() = a; motion_control_set_angle(a); }
int16_t cm_get_diff(void)               { return diff; }
void    cm_set_diff(int16_t d)          { diff = d; motion_control_set_diff(d); }
uint8_t cm_get_angle_enable(void)       { return angle_enable; }
void    cm_set_angle_enable(uint8_t en) { angle_enable = en; motion_control_enable_angle(en); }
int16_t cm_get_plan_distance(void)      { return plan_distance; }
void    cm_set_plan_distance(int16_t d) { plan_distance = d; }
float   cm_get_plan_angle(void)         { return plan_delta; }
void    cm_set_plan_angle(float a)      { plan_delta = a; }
int16_t cm_get_plan_speed(void)          { return plan_speed; }

void control_manager_init(void)
{
    gyro_data_t gyro = gyro_get_data();
    *motion_control_get_target_angle_ptr() = gyro.yaw;
    motion_manager_set_normal(base_speed, gyro.yaw, diff, angle_enable);
}

void control_manager_task(void)
{
    if (!control_manager_tick_flag) {
        return;
    }
    control_manager_tick_flag = 0;

    if (motion_manager_get_state() == MOTION_MANAGER_STATE_NORMAL) {
        motion_control_set_diff(diff);
        motion_control_enable_angle(angle_enable);
        motion_control_set_base_speed(base_speed);
    }
}
