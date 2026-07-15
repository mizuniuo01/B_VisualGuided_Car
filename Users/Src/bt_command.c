/**
 * @file    bt_command.c
 * @brief   蓝牙指令回调实现（get → 加减步长 → set）
 * @author  mizuniuo01
 * @date    2026-07-15
 * @version 1.0.0
 */

#include "bt_command.h"
#include "system.h"
#include "led.h"
#include "motion_manager.h"
#include "control_manager.h"

/* ==================== 速度 PID ==================== */

/**
 * @brief  速度环 KP 增大
 * @param  无
 * @retval 无
 */
void on_spd_kp_up(void)
{
    pid_param_t p;
    pid_get_param(system_pid_speed_left(), &p);
    p.kp += BLT_STEP_SPD_KP;
    pid_set_param(system_pid_speed_left(), &p);
    pid_set_param(system_pid_speed_right(), &p);
}

/**
 * @brief  速度环 KP 减小
 * @param  无
 * @retval 无
 */
void on_spd_kp_down(void)
{
    pid_param_t p;
    pid_get_param(system_pid_speed_left(), &p);
    p.kp -= BLT_STEP_SPD_KP;
    pid_set_param(system_pid_speed_left(), &p);
    pid_set_param(system_pid_speed_right(), &p);
}

/**
 * @brief  速度环 KI 增大
 * @param  无
 * @retval 无
 */
void on_spd_ki_up(void)
{
    pid_param_t p;
    pid_get_param(system_pid_speed_left(), &p);
    p.ki += BLT_STEP_SPD_KI;
    pid_set_param(system_pid_speed_left(), &p);
    pid_set_param(system_pid_speed_right(), &p);
}

/**
 * @brief  速度环 KI 减小
 * @param  无
 * @retval 无
 */
void on_spd_ki_down(void)
{
    pid_param_t p;
    pid_get_param(system_pid_speed_left(), &p);
    p.ki -= BLT_STEP_SPD_KI;
    pid_set_param(system_pid_speed_left(), &p);
    pid_set_param(system_pid_speed_right(), &p);
}

/**
 * @brief  速度环 KD 增大
 * @param  无
 * @retval 无
 */
void on_spd_kd_up(void)
{
    pid_param_t p;
    pid_get_param(system_pid_speed_left(), &p);
    p.kd += BLT_STEP_SPD_KD;
    pid_set_param(system_pid_speed_left(), &p);
    pid_set_param(system_pid_speed_right(), &p);
}

/**
 * @brief  速度环 KD 减小
 * @param  无
 * @retval 无
 */
void on_spd_kd_down(void)
{
    pid_param_t p;
    pid_get_param(system_pid_speed_left(), &p);
    p.kd -= BLT_STEP_SPD_KD;
    pid_set_param(system_pid_speed_left(), &p);
    pid_set_param(system_pid_speed_right(), &p);
}

/* ==================== 角度 PID ==================== */

/**
 * @brief  角度环 KP 增大
 * @param  无
 * @retval 无
 */
void on_ang_kp_up(void)
{
    pid_param_t p;
    pid_get_param(system_pid_angle(), &p);
    p.kp += BLT_STEP_ANG_KP;
    pid_set_param(system_pid_angle(), &p);
}

/**
 * @brief  角度环 KP 减小
 * @param  无
 * @retval 无
 */
void on_ang_kp_down(void)
{
    pid_param_t p;
    pid_get_param(system_pid_angle(), &p);
    p.kp -= BLT_STEP_ANG_KP;
    pid_set_param(system_pid_angle(), &p);
}

/**
 * @brief  角度环 KI 增大
 * @param  无
 * @retval 无
 */
void on_ang_ki_up(void)
{
    pid_param_t p;
    pid_get_param(system_pid_angle(), &p);
    p.ki += BLT_STEP_ANG_KI;
    pid_set_param(system_pid_angle(), &p);
}

/**
 * @brief  角度环 KI 减小
 * @param  无
 * @retval 无
 */
void on_ang_ki_down(void)
{
    pid_param_t p;
    pid_get_param(system_pid_angle(), &p);
    p.ki -= BLT_STEP_ANG_KI;
    pid_set_param(system_pid_angle(), &p);
}

/**
 * @brief  角度环 KD 增大
 * @param  无
 * @retval 无
 */
void on_ang_kd_up(void)
{
    pid_param_t p;
    pid_get_param(system_pid_angle(), &p);
    p.kd += BLT_STEP_ANG_KD;
    pid_set_param(system_pid_angle(), &p);
}

/**
 * @brief  角度环 KD 减小
 * @param  无
 * @retval 无
 */
void on_ang_kd_down(void)
{
    pid_param_t p;
    pid_get_param(system_pid_angle(), &p);
    p.kd -= BLT_STEP_ANG_KD;
    pid_set_param(system_pid_angle(), &p);
}

/* ==================== 基础速度 / 目标角度 ==================== */

/**
 * @brief  基础速度增大
 * @param  无
 * @retval 无
 */
void on_base_spd_up(void)
{
    int16_t spd = control_manager_get_base_speed() + BLT_STEP_BASE_SPD;
    control_manager_set_base_speed(spd);
}

/**
 * @brief  基础速度减小
 * @param  无
 * @retval 无
 */
void on_base_spd_down(void)
{
    int16_t spd = control_manager_get_base_speed() - BLT_STEP_BASE_SPD;
    control_manager_set_base_speed(spd);
}

/**
 * @brief  目标角度增大
 * @param  无
 * @retval 无
 */
void on_target_ang_up(void)
{
    float ang = control_manager_get_target_angle() + BLT_STEP_TARGET_ANG;
    control_manager_set_target_angle(ang);
}

/**
 * @brief  目标角度减小
 * @param  无
 * @retval 无
 */
void on_target_ang_down(void)
{
    float ang = control_manager_get_target_angle() - BLT_STEP_TARGET_ANG;
    control_manager_set_target_angle(ang);
}

/* ==================== 规划距离 / 规划角度 ==================== */

/**
 * @brief  规划距离增大
 * @param  无
 * @retval 无
 */
void on_move_dist_up(void)
{
    int16_t dist = control_manager_get_plan_distance() + BLT_STEP_MOVE_DIST;
    control_manager_set_plan_distance(dist);
}

/**
 * @brief  规划距离减小
 * @param  无
 * @retval 无
 */
void on_move_dist_down(void)
{
    int16_t dist = control_manager_get_plan_distance() - BLT_STEP_MOVE_DIST;
    control_manager_set_plan_distance(dist);
}

/**
 * @brief  规划旋转角度增大
 * @param  无
 * @retval 无
 */
void on_rotate_ang_up(void)
{
    float ang = control_manager_get_plan_angle() + BLT_STEP_ROTATE_ANG;
    control_manager_set_plan_angle(ang);
}

/**
 * @brief  规划旋转角度减小
 * @param  无
 * @retval 无
 */
void on_rotate_ang_down(void)
{
    float ang = control_manager_get_plan_angle() - BLT_STEP_ROTATE_ANG;
    control_manager_set_plan_angle(ang);
}

/**
 * @brief  启动距离移动
 * @param  无
 * @retval 无
 */
void on_move_start(void)
{
    motion_manager_start_move(control_manager_get_plan_distance(),
        control_manager_get_plan_speed());
}

/**
 * @brief  启动角度旋转
 * @param  无
 * @retval 无
 */
void on_rotate_start(void)
{
    motion_manager_start_rotate(control_manager_get_plan_angle(),
        control_manager_get_plan_speed());
}
