#include "PID.h"
#include <math.h>
#include "Serial.h"
/* 应用层速度单位统一为轮子转/秒（wheel rev/s） */

/* 速度环增益：输入轮子速度误差(rev/s)，输出电流指令
 * kp：比例；ki：积分（每秒）；kd：对测量值微分；kff：目标斜率前馈 */
float kp_speed = 7225.7f;                   /* 单位：电流指令/(rev/s) */
float ki_speed = 62832.0f;                  /* 单位：电流指令/((rev/s)*s) */
float kd_speed = 23.04385f;                 /* 单位：电流指令/((rev/s)/s) */
float kff_speed = 11.5f;                    /* 单位：电流指令/((rev/s)/s) */

/* 各电机的目标值，下标 = 电机号 1~MOTOR_COUNT（[0] 不用） */
float speed_target[MOTOR_COUNT + 1U];       /* 单位：轮子转/秒 */

/* 各路速度环输出，作为 C620 的电流指令 */
int16_t current_output[MOTOR_COUNT + 1U];   /* C620电流指令值，±16384≈±20A */

/* 每一路各自的积分、微分状态，不能共用 */
static float speed_ErrorInt[MOTOR_COUNT + 1U];  /* 单位：C620电流指令值 */
static float speed_LastActual[MOTOR_COUNT + 1U];/* 上一周期反馈转速，用于微分 */
static uint8_t speed_D_ready[MOTOR_COUNT + 1U]; /* 0=微分基准未建立，1=已建立 */

/* 输出与积分限幅 */
#define SPEED_IO_LIMIT          16384.0f        /* C620 电流指令上限 */
#define SPEED_INT_LIMIT          17000.0f        /* 积分项上限 */

/* 积分分离阈值（轮子转/秒）：误差大时不积分，避免阶跃时积分堆积 */
#define SPEED_INT_SEPARATION     3.18f

/* 清某一路的状态；motor_no = 1~MOTOR_COUNT */
void PID_Reset(uint8_t motor_no)
{
    if ((motor_no < 1U) || (motor_no > MOTOR_COUNT))
    {
        return;
    }

    speed_target[motor_no] = 0;
    current_output[motor_no] = 0;
    speed_ErrorInt[motor_no] = 0.0f;
    speed_LastActual[motor_no] = 0.0f;
    speed_D_ready[motor_no] = 0;
}

void PID_Init(void)
{
    uint8_t i;

    for (i = 1U; i <= MOTOR_COUNT; i++)
    {
        PID_Reset(i);
    }
}

/*
 * 单路速度环：输入该路目标转速，输出 current_output[motor_no]。
 * 结构：比例(kp) + 积分分离(ki) + 对测量值微分(kd) + 加速度前馈(kff)，
 * 目标侧另有 Control 里的线性斜坡，保证前馈有意义。
 */
void PID_speed(uint8_t motor_no, float Target_speed, float Target_change)
{
    float actual;                               /* 单位：轮子转/秒 */
    float err;                                  /* 单位：轮子转/秒 */
    float derivative;                           /* 单位：C620电流指令值 */
    float feedforward;                          /* 单位：C620电流指令值 */
    float output;

    if ((motor_no < 1U) || (motor_no > MOTOR_COUNT))
    {
        return;
    }

    actual = g_motor[motor_no].wheel_rps;

    /* 目标转速减去反馈转速，得到速度误差 */
    err = Target_speed - actual;

    /* 首拍（含掉线重连第一拍）用当前反馈建立基准，避免微分冲击 */
    if (speed_D_ready[motor_no] == 0)
    {
        speed_LastActual[motor_no] = actual;
        speed_D_ready[motor_no] = 1;
    }

    /* 微分：对测量值微分，目标跳变不会产生微分冲击 */
    derivative = -kd_speed * ((actual - speed_LastActual[motor_no]) / PID_TIME);
    speed_LastActual[motor_no] = actual;

    /* 前馈：目标斜坡带来的已知加速度扰动 */
    feedforward = kff_speed * (Target_change / PID_TIME);

    output = kp_speed * err + speed_ErrorInt[motor_no] + feedforward + derivative;

    /* 积分分离：误差大或输出饱和时，不允许再往同一方向累积；
       但误差与积分反号（需要退积分）时必须放行，否则负载松开的瞬间
       积分卡在正方向、输出又顶在负限幅，转速会一直降不下来 */
    {
        uint8_t allow_int = 1U;

        if (fabsf(err) >= SPEED_INT_SEPARATION)
        {
            if (err * speed_ErrorInt[motor_no] > 0.0f)
            {
                allow_int = 0U;
            }
        }

        if (((output >= SPEED_IO_LIMIT) && (err > 0.0f)) ||
            ((output <= -SPEED_IO_LIMIT) && (err < 0.0f)))
        {
            allow_int = 0U;
        }

        if (allow_int != 0U)
        {
            speed_ErrorInt[motor_no] += ki_speed * err * PID_TIME;
        }
    }

    /* 积分项限幅 */
    if (speed_ErrorInt[motor_no] > SPEED_INT_LIMIT)
        speed_ErrorInt[motor_no] = SPEED_INT_LIMIT;
    if (speed_ErrorInt[motor_no] < -SPEED_INT_LIMIT)
        speed_ErrorInt[motor_no] = -SPEED_INT_LIMIT;

    /* 重算总输出并限幅 */
    output = kp_speed * err + speed_ErrorInt[motor_no] + feedforward + derivative;
    if (output > SPEED_IO_LIMIT)
    {
        output = SPEED_IO_LIMIT;
    }
    else if (output < -SPEED_IO_LIMIT)
    {
        output = -SPEED_IO_LIMIT;
    }

    current_output[motor_no] = (int16_t)output;
}
