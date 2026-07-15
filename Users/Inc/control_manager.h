#ifndef CONTROL_MANAGER_H
#define CONTROL_MANAGER_H

#include <stdint.h>

extern volatile uint8_t control_manager_tick_flag;

void control_manager_init(void);
void control_manager_task(void);

int16_t cm_get_base_speed(void);
void    cm_set_base_speed(int16_t speed);
float   cm_get_target_angle(void);
void    cm_set_target_angle(float angle);
int16_t cm_get_diff(void);
void    cm_set_diff(int16_t diff);
uint8_t cm_get_angle_enable(void);
void    cm_set_angle_enable(uint8_t enable);
int16_t cm_get_plan_distance(void);
void    cm_set_plan_distance(int16_t distance_mm);
float   cm_get_plan_angle(void);
void    cm_set_plan_angle(float delta_deg);
int16_t cm_get_plan_speed(void);

#endif
