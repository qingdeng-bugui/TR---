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

/* 单个电调的反馈与里程状态，下标 = 电机号（1~MOTOR_COUNT） */
typedef struct
{
    volatile float    wheel_rps;    /* 轮子转速反馈，单位：转/秒，正转正、反转负 */
    volatile int16_t  current;      /* 转矩电流反馈，±16384≈±20A */
    volatile uint16_t angle;        /* 转子机械角度，0~8191 */
    volatile float    distance_m;   /* 上电后累计移动距离，单位：m */
    volatile uint32_t last_rx_tick; /* 最近一次收到反馈的 HAL_GetTick，单位：ms */
    volatile uint8_t  online;       /* 1=最近 100ms 内收到过反馈 */

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
