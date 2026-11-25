#ifndef SERIAL_H
#define SERIAL_H

#include "main.h"

/* ==================== C620 电调组（MOTOR_COUNT 路） ==================== */

/*
 * ===== 改这里就能切换电机个数 =====
 * 电调 ID 必须是 1~MOTOR_COUNT（C620 拨码），代码会自动按这个数
 * 生成 PID 数组、CAN 帧、调试帧列数。
 * 上限 8：0x200 一帧带 1~4 号，0x1FF 一帧带 5~8 号。
 *
 * 注意：所有数组都按 [MOTOR_COUNT + 1] 开，下标直接用电机号 1~MOTOR_COUNT，
 * 0 号位空着不用，这样"电机号"和"C620 上的 ID"始终一致，不会差 1。
 */
#ifndef MOTOR_COUNT
#define MOTOR_COUNT                 4U
#endif

#if (MOTOR_COUNT < 1U) || (MOTOR_COUNT > 8U)
#error "MOTOR_COUNT 必须是 1~8"
#endif

#define C620_CONTROL_ID             0x200U   /* 主机 -> 1~4 号电调：8 字节，每 2 字节一个电流（5~8 号用 0x1FF） */
#define C620_FEEDBACK_ID_BASE       0x201U   /* 1 号电调反馈；2~4 号依次 0x202/0x203/0x204 */

/*
 * C620 反馈里的角度和转速都是减速箱前（转子侧）的数据。
 * 轮子装在减速箱输出轴上，所以计算实际距离必须除以减速比。
 *
 * M3508 + C620 的减速比通常为 3591 / 187 = 19.2032。

 */
#define MOTOR_GEAR_RATIO              (268.0f / 17.0f)  /* 减速箱减速比 */
#define MOTOR_WHEEL_CIRCUMFERENCE_M   (2*0.043f*3.1415926f) /* 单位：m */

/* C620协议给转子rpm，应用层统一使用轮子转/秒 */
#define MOTOR_ROTOR_RPM_TO_WHEEL_RPS  (1.0f / (60.0f * MOTOR_GEAR_RATIO))

/*
 * C620 反馈量换算（依据官方说明书 v1.0 的 CAN 通信协议）：
 *   反馈里的转矩电流是原始计数，±16384 对应 ±20A；
 *   M3508 的转矩常数 Kt = 0.3 N·m/A，所以 1 个计数 ≈ 0.3662 mN·m。
 * 这两个系数只用来把反馈量显示成人能看懂的单位，不参与控制。
 */
#define C620_CURRENT_COUNTS_TO_A      (20.0f / 16384.0f)                /* 1 count ≈ 1.2207 mA */
#define C620_CURRENT_COUNTS_TO_NM     (0.3f * C620_CURRENT_COUNTS_TO_A) /* 1 count ≈ 0.3662 mN·m */

/* C620 转子编码器一圈的计数，0~8191 对应 0~360° */
#define C620_ENCODER_COUNTS_PER_REV   8192.0f

/* 单个电调的反馈与里程状态，下标 = 电机号（1~MOTOR_COUNT） */
typedef struct
{
    volatile float    wheel_rps;    /* 轮子转速反馈，单位：转/秒，正转正、反转负 */
    volatile int16_t  current;      /* 转矩电流反馈，±16384≈±20A */
    volatile uint16_t angle;        /* 转子机械角度，0~8191 */
    volatile float    distance_m;   /* 上电后累计移动距离，单位：m */
    volatile uint32_t last_rx_tick; /* 最近一次收到反馈的 HAL_GetTick，单位：ms */
    volatile uint8_t  online;       /* 1=最近 100ms 内收到过反馈 */
    volatile uint8_t  temperature;  /* 电机温度反馈，单位：°C，来自反馈帧 data[6] */
    volatile uint8_t  error_code;   /* 电机错误码，来自反馈帧 data[7]，0=无异常 */

    int32_t  counts;                /* 累计转子编码器计数（含回绕处理） */
    uint16_t last_angle;            /* 上一帧角度，用于求增量 */
    uint8_t  angle_ready;           /* 0=未建立角度基准，1=已建立 */
} MotorFeedback_t;

/* 下标 1~MOTOR_COUNT 有效，g_motor[0] 不使用 */
extern MotorFeedback_t g_motor[MOTOR_COUNT + 1U];

/* 清空全部反馈数据，在 CAN 接收开始前调用 */
void Serial_Init(void);  

/*
 * 帧解析：按 0x201~0x204 分发给对应电机，
 * 读取角度、转速、实际电流，并累计该电机的移动距离。
 */
void Serial_ParseCanFrame(uint16_t can_id,
                          const uint8_t data[8]);  /* data单位：字节 */

/* 100 ms 内收到过该电调反馈就返回 1，否则返回 0（并清掉角度基准）；motor_no = 1~MOTOR_COUNT */
uint8_t Motor_IsOnline(uint8_t motor_no);

/* 任意一个电调在线就返回 1 */
uint8_t Motor_AnyOnline(void);

/* ==================== 反馈量换算与位置控制 ==================== */

/* 电机温度，单位：°C；motor_no = 1~MOTOR_COUNT，越界返回 0 */
uint8_t Motor_GetTemperature(uint8_t motor_no);

/*
 * 电机错误码（官方 C620 说明书 反馈帧 data[7]）：
 *   0 无异常、1 存储芯片无法访问、2 供电电压过高、3 三相线未接入、
 *   4 位置传感器数据丢失、5 温度过高(≥180°C)、7 校准失败、8 过热(≥125°C)
 * 多个故障同时存在时，返回数值更小的（级别更高）。motor_no 越界返回 0。
 */
uint8_t Motor_GetErrorCode(uint8_t motor_no);

/* 反馈转矩电流换算成安培；motor_no 越界返回 0.0f */
float Motor_GetCurrentAmpere(uint8_t motor_no);

/*
 * 反馈转矩电流换算成输出轴扭矩（估算值，单位：N·m）。
 * Kt 取固定 0.3 N·m/A，大电流时磁路饱和会让结果偏大，仅用于调试观察。
 */
float Motor_GetTorqueNm(uint8_t motor_no);

/* 减速箱输出轴累计转角，单位：rad（多圈，上电起累计）；越界返回 0.0f */
float Motor_GetAngleRad(uint8_t motor_no);

/*
 * 里程/转角归零：清掉累计计数与累计距离，下一帧反馈重新建立角度基准。
 * 对应第二份驱动的 djiMotor_Back2Zero()。motor_no = 1~MOTOR_COUNT。
 */
void Motor_ResetPosition(uint8_t motor_no);

/* 全部电机的里程/转角归零 */
void Motor_ResetAllPosition(void);

/* ==================== CAN 硬件层 ==================== */

/*
 * 配置接收滤波器（放行 0x200~0x207，覆盖最多 8 路反馈）、启动 CAN、打开 RX FIFO0 接收中断。
 * 必须在 MX_CAN1_Init()（内部已调用 HAL_CAN_Init）之后调用。
 */
void Serial_CanInit(CAN_HandleTypeDef *hcan);

/*
 * 给全部电调发电流指令：每 4 路一帧（0x200 起，超过 4 路自动用 0x1FF）。
 * currents 下标 = 电机号 1~MOTOR_COUNT（currents[0] 不用），
 * 单位同 C620：±16384 对应 ±20A，函数内部自动限幅。
 * 返回 0=成功，1=未初始化，2=邮箱满或发送失败。
 */
uint8_t Serial_SendCurrents(const int16_t currents[MOTOR_COUNT + 1U]);

#endif /* SERIAL_H */
