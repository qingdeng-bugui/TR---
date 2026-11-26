#include <stdlib.h>

#include "HeaderFiles.h"

/*
 * 速度给定与控制节拍。
 * 目标值 speed_command 由外部接口写入，speed_ref 是按加速度上限斜坡后的值，
 * 两者每路各一份，下标 = 电机号 1~MOTOR_COUNT（[0] 不用），互不影响。
 */
static volatile float speed_command[MOTOR_COUNT + 1U];
static int16_t current_command[MOTOR_COUNT + 1U];
static uint8_t s_mode = MOTOR_MODE_SPEED;
static float speed_ref[MOTOR_COUNT + 1U];

#define SPEED_ACCEL_LIMIT_RPS_S   20.0f      /* 每路缓启动加速度上限，单位：转/秒² */
#define SPEED_CMD_LIMIT_RPS       8.0f       /* 目标轮速限幅，单位：转/秒 */

void M3508_Init(void)
{
    Serial_Init();              /* 清空 C620 反馈数据 */
    Serial_CanInit(&hcan1);     /* 配置滤波器、启动 CAN、开启接收中断 */
    PID_Init();
}

/* 给某一路设定轮子目标转速，单位：转/秒；motor_no 为 1~MOTOR_COUNT（= C620 的 ID） */
void Motor_SetSpeed(uint8_t motor_no, float wheel_rps)
{
    if ((motor_no < 1U) || (motor_no > MOTOR_COUNT))
    {
        return;
    }

    if (wheel_rps > SPEED_CMD_LIMIT_RPS)
    {
        wheel_rps = SPEED_CMD_LIMIT_RPS;
    }
    else if (wheel_rps < -SPEED_CMD_LIMIT_RPS)
    {
        wheel_rps = -SPEED_CMD_LIMIT_RPS;
    }

    speed_command[motor_no] = wheel_rps;
}

void Motor_SetAllSpeed(float wheel_rps)
{
    uint8_t i;

    for (i = 1U; i <= MOTOR_COUNT; i++)
    {
        Motor_SetSpeed(i, wheel_rps);
    }
}

void Motor_Stop(void)
{
    Motor_SetAllSpeed(0.0f);
}

float Motor_GetSpeed(uint8_t motor_no)
{
    if ((motor_no < 1U) || (motor_no > MOTOR_COUNT))
    {
        return 0.0f;
    }
    return speed_ref[motor_no];
}

/*
 * 把某一路的 speed_ref 朝 speed_command 拉近，每周期最多走
 * SPEED_ACCEL_LIMIT_RPS_S*PID_TIME。返回这一周期的目标增量（转/秒），给前馈用。
 */
static float Speed_Ramp(uint8_t motor_no)
{
    const float step = SPEED_ACCEL_LIMIT_RPS_S * PID_TIME;
    float before = speed_ref[motor_no];
    float diff = (float)speed_command[motor_no] - speed_ref[motor_no];

    if (diff > step)
    {
        speed_ref[motor_no] += step;
    }
    else if (diff < -step)
    {
        speed_ref[motor_no] -= step;
    }
    else
    {
        speed_ref[motor_no] = (float)speed_command[motor_no];
    }

    return speed_ref[motor_no] - before;
}

/* 每 5ms 调一次：每路各自算斜坡 → 各自跑速度环 → 统一发一帧电流 */
void Motor_Update(void)
{
    uint8_t i;

    if (s_mode == MOTOR_MODE_CURRENT)
    {
        (void)Serial_SendCurrents(current_command);
        return;
    }

    for (i = 1U; i <= MOTOR_COUNT; i++)
    {
        float delta = Speed_Ramp(i);
        float target = speed_ref[i];

        if (Motor_IsOnline(i))
        {
            speed_target[i] = target;
            PID_speed(i, target, delta);
        }
        else
        {
            PID_Reset(i);                     /* 掉线那一路清零，电流发 0 */
        }
    }

    (void)Serial_SendCurrents(current_output);
}

/* 切换控制模式：速度模式 / 电流模式 */
void Motor_SetMode(uint8_t mode)
{
    s_mode = (mode == MOTOR_MODE_CURRENT) ? MOTOR_MODE_CURRENT : MOTOR_MODE_SPEED;

    if (s_mode == MOTOR_MODE_SPEED)
    {
        uint8_t i;
        for (i = 1U; i <= MOTOR_COUNT; i++)
        {
            PID_Reset(i);                 /* 从电流模式切回来时清掉旧状态 */
        }
    }
}

/* 直接给某一路电流指令（仅电流模式生效），motor_no = 1~MOTOR_COUNT */
void Motor_SetCurrent(uint8_t motor_no, int16_t current)
{
    if ((motor_no < 1U) || (motor_no > MOTOR_COUNT))
    {
        return;
    }

    if (current >  16384) current =  16384;
    if (current < -16384) current = -16384;
    current_command[motor_no] = current;
}

void Motor_SetAllCurrent(int16_t current)
{
    uint8_t i;
    for (i = 1U; i <= MOTOR_COUNT; i++)
    {
        Motor_SetCurrent(i, current);
    }
}
