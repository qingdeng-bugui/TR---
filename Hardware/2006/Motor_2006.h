#ifndef __Motor_2006_H
#define __Motor_2006_H

#include "main.h"

void Motor_2006_Init(void);
uint8_t Motor_2006_SetSpeed(int16_t Current, uint8_t Number);
uint8_t Motor_2006_SetAll(int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4);
float Motor_2006_GetAngle(uint8_t Number);
void Abs_Zero_Init(void);

extern volatile uint32_t Motor_2006_RxCount; //收到的有效反馈帧累计数(调试/丢帧判断)

#endif // !Motor_2006_H
