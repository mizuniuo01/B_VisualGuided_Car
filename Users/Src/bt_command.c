/**
 * @file    bt_command.c
 * @brief   蓝牙指令回调实现（get → 加减步长 → set）
 * @author  mizuniuo01
 * @date    2026-07-15
 */

#include "bt_command.h"
#include "system.h"
#include "led.h"
#include "motion_manager.h"
#include "control_manager.h"

/* ==================== 速度 PID ==================== */

void on_spd_kp_up(void)
{
    pid_param_t p;
    pid_get_param(system_pid_speed_left(), &p);
    p.kp += BLT_STEP_SPD_KP;
    pid_set_param(system_pid_speed_left(), &p);
    pid_set_param(system_pid_speed_right(), &p);
}

void on_spd_kp_down(void)
{
    pid_param_t p;
    pid_get_param(system_pid_speed_left(), &p);
    p.kp -= BLT_STEP_SPD_KP;
    pid_set_param(system_pid_speed_left(), &p);
    pid_set_param(system_pid_speed_right(), &p);
}

void on_spd_ki_up(void)
{
    pid_param_t p;
    pid_get_param(system_pid_speed_left(), &p);
    p.ki += BLT_STEP_SPD_KI;
    pid_set_param(system_pid_speed_left(), &p);
    pid_set_param(system_pid_speed_right(), &p);
}

void on_spd_ki_down(void)
{
    pid_param_t p;
    pid_get_param(system_pid_speed_left(), &p);
    p.ki -= BLT_STEP_SPD_KI;
    pid_set_param(system_pid_speed_left(), &p);
    pid_set_param(system_pid_speed_right(), &p);
}

void on_spd_kd_up(void)
{
    pid_param_t p;
    pid_get_param(system_pid_speed_left(), &p);
    p.kd += BLT_STEP_SPD_KD;
    pid_set_param(system_pid_speed_left(), &p);
    pid_set_param(system_pid_speed_right(), &p);
}

void on_spd_kd_down(void)
{
    pid_param_t p;
    pid_get_param(system_pid_speed_left(), &p);
    p.kd -= BLT_STEP_SPD_KD;
    pid_set_param(system_pid_speed_left(), &p);
    pid_set_param(system_pid_speed_right(), &p);
}

/* ==================== 角度 PID ==================== */

void on_ang_kp_up(void)
{
    pid_param_t p;
    pid_get_param(system_pid_angle(), &p);
    p.kp += BLT_STEP_ANG_KP;
    pid_set_param(system_pid_angle(), &p);
}

void on_ang_kp_down(void)
{
    pid_param_t p;
    pid_get_param(system_pid_angle(), &p);
    p.kp -= BLT_STEP_ANG_KP;
    pid_set_param(system_pid_angle(), &p);
}

void on_ang_ki_up(void)
{
    pid_param_t p;
    pid_get_param(system_pid_angle(), &p);
    p.ki += BLT_STEP_ANG_KI;
    pid_set_param(system_pid_angle(), &p);
}

void on_ang_ki_down(void)
{
    pid_param_t p;
    pid_get_param(system_pid_angle(), &p);
    p.ki -= BLT_STEP_ANG_KI;
    pid_set_param(system_pid_angle(), &p);
}

void on_ang_kd_up(void)
{
    pid_param_t p;
    pid_get_param(system_pid_angle(), &p);
    p.kd += BLT_STEP_ANG_KD;
    pid_set_param(system_pid_angle(), &p);
}

void on_ang_kd_down(void)
{
    pid_param_t p;
    pid_get_param(system_pid_angle(), &p);
    p.kd -= BLT_STEP_ANG_KD;
    pid_set_param(system_pid_angle(), &p);
}

/* ==================== 基础速度 / 目标角度 ==================== */

void on_base_spd_up(void)
{
    cm_set_base_speed(cm_get_base_speed() + BLT_STEP_BASE_SPD);
}

void on_base_spd_down(void)
{
    cm_set_base_speed(cm_get_base_speed() - BLT_STEP_BASE_SPD);
}

void on_target_ang_up(void)
{
    cm_set_target_angle(cm_get_target_angle() + BLT_STEP_TARGET_ANG);
}

void on_target_ang_down(void)
{
    cm_set_target_angle(cm_get_target_angle() - BLT_STEP_TARGET_ANG);
}

/* ==================== 规划距离 / 规划角度 ==================== */

void on_move_dist_up(void)
{
    cm_set_plan_distance(cm_get_plan_distance() + BLT_STEP_MOVE_DIST);
}

void on_move_dist_down(void)
{
    cm_set_plan_distance(cm_get_plan_distance() - BLT_STEP_MOVE_DIST);
}

void on_rotate_ang_up(void)
{
    cm_set_plan_angle(cm_get_plan_angle() + BLT_STEP_ROTATE_ANG);
}

void on_rotate_ang_down(void)
{
    cm_set_plan_angle(cm_get_plan_angle() - BLT_STEP_ROTATE_ANG);
}

void on_move_start(void)
{
    motion_manager_start_move(cm_get_plan_distance(), cm_get_base_speed());
}

void on_rotate_start(void)
{
    motion_manager_start_rotate(cm_get_plan_angle(), cm_get_base_speed());
}
