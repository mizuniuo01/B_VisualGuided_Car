#ifndef CONTROL_MANAGER_H
#define CONTROL_MANAGER_H

#include <stdint.h>

/* 路口控制距离（mm） */
typedef enum {
    JUNCTION_DIST_STRAIGHT_MM  = 500,
    JUNCTION_DIST_TURN_MM      = 400,
    JUNCTION_DIST_GREEN_REDUCE = 200,
} control_junction_dist_t;

/* 路口旋转角度（度，浮点常量必须用宏） */
#define JUNCTION_ANGLE_RIGHT_DEG -90.0f /* 右转角度 */
#define JUNCTION_ANGLE_LEFT_DEG  90.0f  /* 左转角度 */

/* 顶层控制状态 */
typedef enum {
    CONTROL_MANAGER_STATE_STOP = 0,
    CONTROL_MANAGER_STATE_RUNNING,
} control_manager_state_t;

/* 运行子状态 */
typedef enum {
    CONTROL_RUN_NORMAL = 0,
    CONTROL_RUN_JUNCTION_MOVE,
    CONTROL_RUN_JUNCTION_ROTATE,
    CONTROL_RUN_WAIT_GREEN,
} control_run_substate_t;

/* 普通闭环参数 */
typedef struct {
    int16_t base_speed;    /* 基础速度（count/10ms） */
    uint8_t angle_enable;  /* 角度环使能 */
} control_normal_params_t;

/* 运动规划参数 */
typedef struct {
    int16_t distance_mm;  /* 移动距离（mm） */
    float   delta_deg;    /* 旋转角度（度） */
    int16_t speed;        /* 规划速度（count/10ms） */
} control_plan_params_t;

extern volatile uint8_t control_manager_tick_flag;

void control_manager_init(void);
void control_manager_task(void);

void control_manager_set_running(uint8_t run);
control_manager_state_t control_manager_get_state(void);
control_run_substate_t  control_manager_get_substate(void);

const control_normal_params_t *control_manager_get_normal_params(void);
void control_manager_set_normal_params(const control_normal_params_t *p);
const control_plan_params_t *control_manager_get_plan_params(void);
void control_manager_set_plan_params(const control_plan_params_t *p);

#endif
