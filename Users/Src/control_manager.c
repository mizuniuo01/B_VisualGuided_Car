/**
 * @file    control_manager.c
 * @brief   顶层控制调度（控制参数缓存 + 下发）
 * @author  mizuniuo01
 * @date    2026-07-15
 */

#include "control_manager.h"
#include "motion_control.h"

static int16_t base_speed     = 0;
static float  target_angle    = 0.0f;
static int16_t diff           = 0;
static uint8_t angle_enable   = 0;
static int16_t plan_distance  = 500;
static float  plan_delta      = 90.0f;
static int16_t plan_speed     = 50;

int16_t cm_get_base_speed(void)            { return base_speed; }
void    cm_set_base_speed(int16_t speed)   { base_speed = speed; motion_control_set_base_speed(speed); }
float   cm_get_target_angle(void)          { return target_angle; }
void    cm_set_target_angle(float angle)   { target_angle = angle; motion_control_set_angle(angle); }
int16_t cm_get_diff(void)                 { return diff; }
void    cm_set_diff(int16_t d)            { diff = d; motion_control_set_diff(d); }
uint8_t cm_get_angle_enable(void)         { return angle_enable; }
void    cm_set_angle_enable(uint8_t en)   { angle_enable = en; motion_control_enable_angle(en); }
int16_t cm_get_plan_distance(void)        { return plan_distance; }
void    cm_set_plan_distance(int16_t mm)  { plan_distance = mm; }
float   cm_get_plan_angle(void)           { return plan_delta; }
void    cm_set_plan_angle(float deg)      { plan_delta = deg; }
