#ifndef BT_COMMAND_H
#define BT_COMMAND_H

/* 蓝牙调参步长 */
#define BLT_STEP_SPD_KP     1.0f
#define BLT_STEP_SPD_KI     0.5f
#define BLT_STEP_SPD_KD     1.0f
#define BLT_STEP_ANG_KP     1.0f
#define BLT_STEP_ANG_KI     0.1f
#define BLT_STEP_ANG_KD     1.0f
#define BLT_STEP_BASE_SPD   1
#define BLT_STEP_TARGET_ANG 5.0f
#define BLT_STEP_MOVE_DIST  50
#define BLT_STEP_ROTATE_ANG 5.0f

void on_led1_toggle_cmd(void);

void on_spd_kp_up(void);
void on_spd_kp_down(void);
void on_spd_ki_up(void);
void on_spd_ki_down(void);
void on_spd_kd_up(void);
void on_spd_kd_down(void);

void on_ang_kp_up(void);
void on_ang_kp_down(void);
void on_ang_ki_up(void);
void on_ang_ki_down(void);
void on_ang_kd_up(void);
void on_ang_kd_down(void);

void on_base_spd_up(void);
void on_base_spd_down(void);
void on_target_ang_up(void);
void on_target_ang_down(void);

void on_move_dist_up(void);
void on_move_dist_down(void);
void on_rotate_ang_up(void);
void on_rotate_ang_down(void);
void on_move_start(void);
void on_rotate_start(void);

#endif
