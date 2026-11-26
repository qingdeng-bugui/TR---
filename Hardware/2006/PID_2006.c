#include "PID_2006.h"
#include "tim.h"
#include "Motor_2006.h"
#include "Abs_Encoder.h"

PIDController g_pid[4]; //存放pid

static float Target_Chassis[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //各电机的目标转向角度
static uint8_t ZeroAngle_Flag[4] = {1, 1, 1, 1}; //各电机是否已完成上电归零
static float ZeroAngle_Offset[4] = {0.0f};        //各电机上电归零时的累计角度

/**
 * @brief 将角度限制在 [0, 360) 范围内
 * @param angle 输入角度
 * @return 限制后的角度
 */
static float Wrap360f(float angle)
{
    while (angle >= 360.0f)
    {
        angle -= 360.0f;
    }
    while (angle < 0.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 将角度限制在 [-180, 180) 范围内，让电机在最短路径上旋转
 * @param angle 输入角度
 * @return 限制后的角度
 */
static float Wrap180f(float angle)
{
    angle = Wrap360f(angle);
    if (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    return angle;
}

// 初始化PID
void PID_2006_Init(PIDController *pid)
{
    pid->Kp = 200.0;
    pid->Ki = 0.0;
    pid->Kd = 3200.0;
    pid->error = 0.0f;
    pid->Last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->Current_Angle = 0.0f;
    pid->integral_Max = 500.0f;
    pid->Out = 0.0f;
}

/**
 * @brief 设置目标底盘角度
 * @param Angle 目标角度 (0~360度)
 * @param Number 电机编号 (1~4)
 * @return void
 */
void Set_Target_ChassisAngle(float Angle, uint8_t Number)
{
    if ((Number < 1U) || (Number > 4U))
    {
        return;
    }
    Target_Chassis[Number - 1U] = Wrap360f(Angle);
}

/**
 * @brief 请求某一路重新做上电归零 (由 Abs_Zero_Init 调用)
 * @param Number 电机编号 (1~4)
 * @return void
 */
void PID_Request_Zero(uint8_t Number)
{
    if ((Number < 1U) || (Number > 4U))
    {
        return;
    }
    ZeroAngle_Flag[Number - 1U] = 1U;
}

// 计算PID输出
static void PID_Cala(uint8_t Number)
{
    PIDController *pid = &g_pid[Number - 1];
    uint8_t idx = (uint8_t)(Number - 1);
    uint8_t is_zeroing = ZeroAngle_Flag[idx];

    // 读取编码器的角度
    if (is_zeroing == 1)
    {
        float enc_angle = Abs_Encoder_Function(Number);
        if (enc_angle < 0.0f) //编码器读失败(SPI/CRC)时不能当成 0 度, 本周期不动
        {
            pid->Current_Angle = 0.0f;
            pid->Out = 0.0f;
            return;
        }
        pid->Current_Angle = enc_angle;
        if ((enc_angle <= 1.0f) || (enc_angle >= 359.0f))//编码器归零完成, 记录归零时的角度偏移（误差约为+-0.5度）
        {
            ZeroAngle_Offset[idx] = Motor_2006_GetAngle(Number);
            ZeroAngle_Flag[idx] = 0;
        }
    }
    else
    {
        pid->Current_Angle = Motor_2006_GetAngle(Number) - ZeroAngle_Offset[idx];
    }

    // 计算误差: 转向轴目标角, 取最短路径(±180 度)后换算成转子角(63:1)
    float err;
    if (is_zeroing == 1)
    {
        err = 0.0f - pid->Current_Angle;
    }
    else
    {
        err = Wrap180f(Target_Chassis[idx] - Wrap360f(pid->Current_Angle / 63.0f)) * 63.0f;// 转向轴目标角换算成转子角
    }
    pid->Last_error = pid->error;
    pid->error = err;

    // 计算微分项
    pid->derivative = pid->error - pid->Last_error;

    // 先算不含新积分的输出，判断是否饱和（抗积分饱和）
    float out = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * pid->derivative;

    if ((out < 10000.0f) && (out > -10000.0f))
    {
        // 计算积分项
        pid->integral += pid->error;

        // 积分项限幅，防止积分饱和
        if (pid->integral > pid->integral_Max)
        {
            pid->integral = pid->integral_Max;
        }
        else if (pid->integral < -pid->integral_Max)
        {
            pid->integral = -pid->integral_Max;
        }

        out = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * pid->derivative;
    }

    // 输出限幅
    if (out > 10000.0f)
    {
        out = 10000.0f;
    }
    else if (out < -10000.0f)
    {
        out = -10000.0f;
    }

    pid->Out = out;

    //上电归零阶段限流, 避免四台电机同时以满电流猛冲到零点
    if (is_zeroing == 1)
    {
        if (pid->Out > 1500.0f)
        {
            pid->Out = 1500.0f;
        }
        else if (pid->Out < -1500.0f)
        {
            pid->Out = -1500.0f;
        }
    }
}

//用回调函数完成PID
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        int16_t Out[4];
        uint8_t i;

        static uint32_t last_rx_count = 0U;
        static uint16_t no_fb_ms = 0U;

        (void)Motor_2006_GetAngle(1);

        if (Motor_2006_RxCount == last_rx_count)
        {
            if (no_fb_ms < 60000U)
            {
                no_fb_ms++;
            }
        }
        else
        {
            last_rx_count = Motor_2006_RxCount;
            no_fb_ms = 0U;
        }  //先把 CAN 接收 FIFO 清空

        for (i = 1; i <= 4; i++)
        {
            PID_Cala(i);
            Out[i - 1] = (int16_t)g_pid[i - 1].Out;
        }

        /* CAN 反馈丢失(20ms 无新帧) -> 四路输出清零, 防止拿旧角度继续出力 */
        if (no_fb_ms >= 20U)
        {
            Out[0] = 0;
            Out[1] = 0;
            Out[2] = 0;
            Out[3] = 0;
        }

        Motor_2006_SetAll(Out[0], Out[1], Out[2], Out[3]);
    }
}
