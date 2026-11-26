#ifndef __PID_2006_H
#define __PID_2006_H

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error; // 当前误差
    float Last_error; // 上一次误差
    float integral; // 误差累积
    float derivative; // 误差变化率
    float Current_Angle; // 编码器反馈角度
    float Out; //PID输出
    float integral_Max; //积分限幅阈值
} PIDController;

extern PIDController g_pid[4]; //存放pid

void Set_Target_ChassisAngle(float Angle, uint8_t Number);
void PID_Request_Zero(uint8_t Number);
void PID_2006_Init(PIDController *pid);

#endif
