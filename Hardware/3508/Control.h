#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/*
 *   1 号 → 反馈 0x201、下发帧 0x200 的第 1 个槽
 *   2 号 → 反馈 0x202、下发帧 0x200 的第 2 个槽
 *   3 号 → 反馈 0x203、0x200 第 3 个槽
 *   4 号 → 反馈 0x204、0x200 第 4 个槽
 *
 * Motor_SetSpeed(1, 1.0f);   → 只有 1 号轮子跑到 1 转/秒，其余不动
 * Motor_SetSpeed(4, -0.8f);  → 4 号轮子反转 0.8 转/秒
 * Motor_SetAllSpeed(1.2f);   → 所有轮子一起 1.2 转/秒
 * Motor_Stop();              → 全部停止
 */
void Motor_SetSpeed(uint8_t motor_no, float wheel_rps);

/* 所有路设成同一个目标速度 */
void Motor_SetAllSpeed(float wheel_rps);

/* 全部停转 */
void Motor_Stop(void);

/* 某一路斜坡后的实际给定，单位：轮子转/秒（调试用） */
float Motor_GetSpeed(uint8_t motor_no);

/* 控制节拍：由 TIM1 的 5ms 中断调用，算斜坡 → 逐路速度环 → 发 CAN */
void Motor_Update(void);

/* ================= 电流模式（绕过速度环） =================
 * 切到电流模式后，Motor_Update() 直接把 Motor_SetCurrent 给的值下发，
 *
 *   Motor_SetMode(MOTOR_MODE_CURRENT);
 *   Motor_SetCurrent(1, 600);     → 1 号电机恒定电流 600 counts ≈ 0.73A
 */
#define MOTOR_MODE_SPEED    0U              /* 速度模式 */
#define MOTOR_MODE_CURRENT  1U              /* 电流模式 */

void Motor_SetMode(uint8_t mode);
void Motor_SetCurrent(uint8_t motor_no, int16_t current);
void Motor_SetAllCurrent(int16_t current);

#endif /* CONTROL_H */
