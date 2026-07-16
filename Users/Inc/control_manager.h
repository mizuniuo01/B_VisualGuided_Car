#ifndef CONTROL_MANAGER_H
#define CONTROL_MANAGER_H

#include <stdint.h>

/* 分段移动距离（mm） */
#define CONTROL_SEGMENT_DISTANCE_MM 685
/* 转弯角度（度） */
#define CONTROL_TURN_ANGLE_RIGHT_DEG -90.0f /* 右转角度 */
#define CONTROL_TURN_ANGLE_LEFT_DEG 90.0f  /* 左转角度 */

/* 黑线冷却距离 50cm */
#define BLACK_LINE_COOLDOWN_MM 500

/* 方向补偿单位长度（mm） */
#define CONTROL_COMPENSATION_UNIT_MM 100

/* 默认旋转角度（度） */
#define CONTROL_DEFAULT_DELTA_DEG 90.0f

/* 默认移动速度（count/10ms） */
#define CONTROL_DEFAULT_SPEED 20

/* 顶层控制状态 */
typedef enum {
    CONTROL_MANAGER_STATE_STOP = 0,
    CONTROL_MANAGER_STATE_RUNNING,
} control_manager_state_t;

/* 运行子状态 */
typedef enum {
    CONTROL_RUN_MOVE = 0,
    CONTROL_RUN_TURN,
    CONTROL_RUN_BLACK_LINE_WAIT,
} control_run_substate_t;

/* 普通闭环参数（保留，bt_command 使用） */
typedef struct {
    int16_t base_speed;    /* 基础速度（count/10ms） */
    uint8_t angle_enable;  /* 角度环使能 */
} control_normal_params_t;

/* 运动规划参数（保留：distance_mm 作分段距离，speed 作分段速度） */
typedef struct {
    int16_t distance_mm;  /* 分段移动距离（mm） */
    float delta_deg;    /* 旋转角度（度） */
    int16_t speed;        /* 规划速度（count/10ms） */
} control_plan_params_t;

/* 方向坐标系 */
typedef enum {
    DIR_STATE_BACK = 0,     /* 后 */
    DIR_STATE_LEFT = 1,     /* 左 */
    DIR_STATE_FRONT = 2,    /* 前 */
    DIR_STATE_RIGHT = 3,    /* 右 */
} dir_state_t;

/* 方向数量 */
#define DIR_NUM 4

extern volatile uint8_t control_manager_tick_flag;

void control_manager_init(void);
void control_manager_task(void);

void control_manager_set_running(uint8_t run);
control_manager_state_t control_manager_get_state(void);
control_run_substate_t control_manager_get_substate(void);

const control_normal_params_t *control_manager_get_normal_params(void);
void control_manager_set_normal_params(const control_normal_params_t *p);
const control_plan_params_t *control_manager_get_plan_params(void);
void control_manager_set_plan_params(const control_plan_params_t *p);
void control_manager_set_manual_override(uint8_t enable);

#endif
