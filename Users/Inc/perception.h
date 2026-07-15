#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <stdint.h>

/* 感知参数 */
#define FEEDFORWARD_COEFF 0.5f   /* 前馈系数，视觉偏差→差速缩放因子 */
#define OBSTACLE_THRESH_MM 50.0f  /* 障碍物距离阈值（mm） */

typedef struct {
    uint8_t junction_flag;  /* 1: 是路口, 0: 不是路口 */
    uint8_t direction;      /* 0: 直行, 1: 右转, 2: 左转 */
    uint8_t green;          /* 1: 绿灯, 0: 非绿灯 */
    int16_t diff;           /* 视觉偏差值，单位像素，正数表示偏左，负数表示偏右 */
    uint8_t all_black_flag; /* 1: 全黑, 0: 非全黑 */
    uint8_t obstacle_flag;  /* 1: 有障碍物, 0: 无障碍物 */
} perception_data_t;

void perception_init(void);
void perception_task(void);
perception_data_t *perception_get_data(void);

#endif
