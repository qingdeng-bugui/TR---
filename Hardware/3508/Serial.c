#include "Serial.h"

/* 4 个电调的反馈状态，索引 0~3 对应 1~4 号电调 */
MotorFeedback_t g_motor[MOTOR_COUNT + 1U];

/* 保存外部传进来的 CAN 句柄，换 CAN2/CAN3 时只要改这里传参即可 */
static CAN_HandleTypeDef *s_hcan = 0;

/* 清空反馈数据，在 CAN 接收开始前调用 */
void Serial_Init(void)
{
    uint8_t i;

    for (i = 1U; i <= MOTOR_COUNT; i++)
    {
        g_motor[i].wheel_rps = 0.0f;
        g_motor[i].current = 0;
        g_motor[i].angle = 0;
        g_motor[i].distance_m = 0.0f;
        g_motor[i].last_rx_tick = 0;
        g_motor[i].online = 0;
        g_motor[i].counts = 0;
        g_motor[i].last_angle = 0;
        g_motor[i].angle_ready = 0;
    }
}

/*
 * 解析一帧 C620 反馈帧。
 * C620 用大端，16 位数据都是高字节在前。
 */
void Serial_ParseCanFrame(uint16_t can_id, const uint8_t data[8])
{
    MotorFeedback_t *m;
    uint16_t index;
    int32_t delta;
    int16_t rotor_rpm;

    if (can_id < C620_FEEDBACK_ID_BASE)
    {
        return;
    }

    index = (uint16_t)(can_id - C620_FEEDBACK_ID_BASE) + 1U;
    if (index > MOTOR_COUNT)
    {
        return;
    }

    m = &g_motor[index];

    /* data[0..1]：转子机械角度，0~8191，一圈 8192 个计数 */
    m->angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

    /* data[2..3]：C620协议转子转速；换算为轮子转/秒 */
    rotor_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    m->wheel_rps = (float)rotor_rpm * MOTOR_ROTOR_RPM_TO_WHEEL_RPS;

    /* data[4..5]：实际转矩电流，±16384≈±20A */
    m->current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

    m->last_rx_tick = HAL_GetTick();            /* HAL_GetTick单位：ms */
    m->online = 1;

    /*
     * 第一帧只建立角度基准。否则如果上电时角度不是 0，
     * 第一次计算会产生一个虚假的大距离。
     */
    if (m->angle_ready == 0)
    {
        m->last_angle = m->angle;
        m->angle_ready = 1;
        delta = 0;
    }
    else
    {
        delta = (int32_t)m->angle - (int32_t)m->last_angle;

        /* 13位编码器按8192计数回绕 */
        if (delta > 4096)
        {
            delta -= 8192;
        }
        else if (delta < -4096)
        {
            delta += 8192;
        }
    }

    m->last_angle = m->angle;
    m->counts += delta;

    /* 累计距离：转子圈数 / 减速比 × 轮子周长 */
    m->distance_m = ((float)m->counts / 8192.0f) *
                    MOTOR_WHEEL_CIRCUMFERENCE_M / MOTOR_GEAR_RATIO;
}

/* 100 ms 内收到过该电调反馈就算在线 */
uint8_t Motor_IsOnline(uint8_t motor_no)
{
    MotorFeedback_t *m;

    if ((motor_no < 1U) || (motor_no > MOTOR_COUNT))
    {
        return 0;
    }

    m = &g_motor[motor_no];

    if (m->last_rx_tick == 0)
    {
        m->angle_ready = 0;
        m->online = 0;
        return 0;
    }

    if ((HAL_GetTick() - m->last_rx_tick) > 100)
    {
        m->angle_ready = 0;
        m->online = 0;
        return 0;
    }

    return 1;
}

/* 任意一个电调在线就算在线 */
uint8_t Motor_AnyOnline(void)
{
    uint8_t i;

    for (i = 1U; i <= MOTOR_COUNT; i++)
    {
        if (Motor_IsOnline(i))
        {
            return 1;
        }
    }
    return 0;
}

/* ==================== CAN 硬件层 ==================== */

/*
 * 滤波器 + 启动 + 开接收中断。
 * ID = 0x200、掩码 = 0x7F8，放行 0x200~0x207，
 * 覆盖 1~4 号电调的反馈帧（0x201~0x204），其它 ID 由硬件丢弃。
 */
void Serial_CanInit(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};

    s_hcan = hcan;

    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = (uint16_t)(C620_CONTROL_ID << 5);
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = (uint16_t)(0x7F8U << 5);
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return;
    }
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return;
    }

    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*
 * 全部电调的电流指令 -> 每 4 路一帧。
 * 第 1 帧 0x200 带 1~4 号，第 2 帧 0x1FF 带 5~8 号。
 * 帧内 data[0..1] 是本组第 1 个电机，data[2..3] 是第 2 个，依次类推。
 */
uint8_t Serial_SendCurrents(const int16_t currents[MOTOR_COUNT + 1U])
{
    uint8_t group;
    uint8_t result = 0;

    if (s_hcan == 0)
    {
        return 1;
    }

    for (group = 0; (uint16_t)(group * 4U) < MOTOR_COUNT; group++)
    {
        CAN_TxHeaderTypeDef tx = {0};
        uint8_t data[8] = {0};
        uint32_t mailbox;
        uint8_t i;

        /* 三个邮箱都满了就先丢这一帧，避免死等 */
        if (HAL_CAN_GetTxMailboxesFreeLevel(s_hcan) == 0)
        {
            return 2;
        }

        for (i = 0; i < 4U; i++)
        {
            uint16_t index = (uint16_t)(group * 4U + i) + 1U;
            int16_t current;

            if (index > MOTOR_COUNT)
            {
                break;                       /* 本组不满 4 个，剩余字节保持 0 */
            }

            current = currents[index];

            /* ±16384 ≈ ±20A，超出范围先限幅 */
            if (current >  16384) current =  16384;
            if (current < -16384) current = -16384;

            /* 大端：高字节在前 */
            data[2U * i]      = (uint8_t)(current >> 8);
            data[2U * i + 1U] = (uint8_t)(current & 0xFF);
        }

        tx.StdId  = (group == 0U) ? C620_CONTROL_ID
                                  : (uint16_t)(C620_CONTROL_ID - group);
        tx.ExtId  = 0;
        tx.IDE    = CAN_ID_STD;
        tx.RTR    = CAN_RTR_DATA;
        tx.DLC    = 8;
        tx.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_AddTxMessage(s_hcan, &tx, data, &mailbox) != HAL_OK)
        {
            result = 2;
        }
    }

    return result;
}

/*
 * 覆写 HAL 的弱函数：中断里只做"取数据 + 分发"，解析交给上面的
 * Serial_ParseCanFrame()。一次把 FIFO 里的帧全部读完，防止溢出。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK)
        {
            break;
        }
        if (rx.IDE == CAN_ID_STD)
        {
            Serial_ParseCanFrame((uint16_t)rx.StdId, data);
        }
    }
}
