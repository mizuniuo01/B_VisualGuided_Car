/**
 * @file    system.c
 * @brief   硬件资源注册中心 — 句柄定义 + 驱动初始化
 * @author  mizuniuo01
 * @date    2026-06-01
 * @version 1.0.0
 * @note    本项目所有硬件句柄的唯一定义处，外部只能通过 system.h 的 getter 访问。
 * @note    新增模块：定义 static 句柄 → 添加 getter → system_init 中调用 xxx_init。
 * @note    平台初始化（SYSCFG_DL_init/时钟/NVIC/__enable_irq）在 main.c 中，
 *          system_init 只做驱动层 init，由 main.c 在平台就绪后调用。
 * @warning getter 返回的是句柄指针，调用方不可 free 或修改句柄内部字段。
 *
 * @usage
 * 项目级模板文件。复制到目标工程后，按实际使用的模块增删：
 *
 * 1. 句柄实体：保留用到的，删除或注释不用的
 * 2. getter 函数：与句柄实体一一对应
 * 3. system_init()：只写驱动 xxx_init()，平台初始化（SYSCFG_DL_init 等）在 main.c
 *
 * 外部文件通过 system.h 的 getter 拿句柄：
 *
 * led_toggle(system_led1());
 * motor_set_speed(system_motor_left(), 500);
 */

#include "system.h"
#include "gpio.h"

/* 句柄实体（全部 static，外部不可直接访问） */
static led_handle_t led1;
static led_handle_t led2;
static led_handle_t led3;
static led_handle_t led4;
static buzzer_handle_t buzzer;
static motor_handle_t motor_left;
static motor_handle_t motor_right;
/*static pid_controller_t pid_speed_left;
static pid_controller_t pid_speed_right;*/

/* getter：只返回指针，不暴露实体 */

led_handle_t *system_led1(void)
{
    return &led1;
}

led_handle_t *system_led2(void)
{
    return &led2;
}

led_handle_t *system_led3(void)
{
    return &led3;
}

led_handle_t *system_led4(void)
{
    return &led4;
}

buzzer_handle_t *system_buzzer(void)
{
    return &buzzer;
}

motor_handle_t *system_motor_left(void)
{
    return &motor_left;
}

motor_handle_t *system_motor_right(void)
{
    return &motor_right;
}
/*
pid_controller_t *system_pid_speed_left(void)
{
    return &pid_speed_left;
}

pid_controller_t *system_pid_speed_right(void)
{
    return &pid_speed_right;
}
*/
/**
 * @brief  初始化所有驱动模块（在 main.c 进行平台初始化）
 * @param  无
 * @retval 无
 */
void system_init(void)
{
    /* GPIO 控制类 */
    led_cfg_t led1_cfg = {.port = GPIOB, .pin = led1_Pin, .active_level = 1};
    led_init(&led1, &led1_cfg);
    led_cfg_t led2_cfg = {.port = GPIOB, .pin = led2_Pin, .active_level = 1};
    led_init(&led2, &led2_cfg);
    led_cfg_t led3_cfg = {.port = GPIOB, .pin = led3_Pin, .active_level = 1};
    led_init(&led3, &led3_cfg);
    led_cfg_t led4_cfg = {.port = GPIOB, .pin = led4_Pin, .active_level = 1};
    led_init(&led4, &led4_cfg);

    buzzer_cfg_t buzzer_cfg = {.port = GPIOA, .pin = buzzer_Pin, .active_level = 1};
    buzzer_init(&buzzer, &buzzer_cfg);

    /* 通信单实例（直接 init，不走句柄） */
    /* blueteeth_init(UART_BLUETEETH_INST); */
    /* oled_init(I2C_OLED_INST); */
}
