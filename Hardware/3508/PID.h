#ifndef PID_H
#define PID_H

#include <stdint.h>

#include "Serial.h"                             /* MOTOR_COUNT */

/* 控制周期，与 TIM1 的 5ms 更新周期保持一致 */
#define PID_TIME              0.005f            /* 单位：s */

/* 速度环：每路一个目标、一个电流输出，下标 = 电机号 1~MOTOR_COUNT（[0] 不用） */
extern float speed_target[MOTOR_COUNT + 1U];        /* 单位：轮子转/秒 */
extern int16_t current_output[MOTOR_COUNT + 1U];    /* C620电流指令值 */

/* 速度环增益，所有电机共用，运行时可改，便于在线调参 */
extern float kp_speed;
extern float ki_speed;
extern float kd_speed;
extern float kff_speed;

/* 清零全部电机的误差/积分/微分状态 */
void PID_Init(void);

/* 只清某一路的状态（该路掉线或重新上线时用）；motor_no = 1~MOTOR_COUNT */
void PID_Reset(uint8_t motor_no);

/*
 * 单路速度环：输入轮子目标转速，输出 current_output[motor_no]（C620电流指令）。
 * 不直接发 CAN，所有路算完后由 Control 统一发一帧 0x200。
 */
void PID_speed(uint8_t motor_no,                  /* 电机号 1~MOTOR_COUNT */
               float Target_speed,                /* 单位：轮子转/秒 */
               float Target_change);              /* 每周期目标变化，单位：转/秒 */

#endif /* PID_H */
