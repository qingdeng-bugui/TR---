#include "Abs_Encoder.h"
#include "spi.h"
#include "Motor_2006.h" /* Motor_2006_RxCount */
#include "PID_2006.h"   /* PID_Request_Zero */


/**
 * ============================================================
 * 功能：读取 MT6701 SSI 模式下的 14bit 绝对角度（SPI4，Mode 2）
 * ============================================================
 * 接线（1~4 号编码器片选分别对应）：
 *   编码器 DO  (A)  ---- PE5 J1 (SPI4_MISO，上拉输入)
 *   编码器 CLK (B)  ---- PE12 K2 (SPI4_SCK)
 *   编码器 CSN (Z)  ---- PE4 J2/ PC2 L1/ PC3 M1/ PB0 L2(GPIO 软件片选，低有效)
 *   编码器 VCC ------- 3.3V / 5V
 *   编码器 GND ------- GND
 *
 * SPI 参数：
 *   主机、8bit、MSB 先行、Mode 2（CPOL=1, CPHA=0）、约 1.3MHz、软件 NSS
 *
 * SSI 帧：
 *   D[13:0] 角度原始值 + Mg[3:0] 磁场状态 + CRC[5:0] 校验码
 *   CRC 多项式 x^6+x+1。

 *  Abs_Encoder_Function();反馈角度

 * ============================================================ */

/* 忙等延时 */
static void Encoder_DelayUs(uint32_t us)
{
    volatile uint32_t n = us * 42U;

    while (n--)
    {
        __NOP();
    }
}

/**
 * @brief 初始化绝对编码器归零
 * @note 先等 CAN 反馈(最多 300 ms), 再等 4 路绝对编码器都可读(每路最多 500 ms)
 * @note 随后请求 4 路重新归零: 归零由 PID 完成, 完成后 PID 侧置 ZeroAngle_Flag[i]=0
 *       并记录 ZeroAngle_Offset[i] = 各电机归零时的累计角度
 * @retval void
 */
void Abs_Zero_Init(void)
{
    uint8_t n;
    uint16_t t;
    for (t = 0U; (t < 300U) && (Motor_2006_RxCount == 0U); t++)
    {
        HAL_Delay(1);
    }

    for (n = 1U; n <= 4U; n++)
    {
        // 等待编码器可读，最多等待 500 ms
        for (t = 0U; t < 100U; t++)
        {
            float enc = Abs_Encoder_Function(n);
            if (enc >= 0.0f)
            {
                break;
            }
            HAL_Delay(5);
        }
        PID_Request_Zero(n);
    }
}


/**
 * @brief 读取一帧原始数据：片选拉低 -> 读 4 字节(32 个时钟) -> 片选拉高
 * @param Number 1~4
 * @param rx 4 字节接收缓冲区
 * @return HAL_OK 表示 SPI 传输完成
 */
static HAL_StatusTypeDef Encoder_Read(uint8_t Number, uint8_t rx[4])
{
    static const struct
    {
        GPIO_TypeDef *port;
        uint16_t pin;
    } cs[] = {
        {GPIOE, GPIO_PIN_4},
        {GPIOC, GPIO_PIN_2},
        {GPIOC, GPIO_PIN_3},
        {GPIOB, GPIO_PIN_0},
    };
    const uint8_t tx[4] = {0, 0, 0, 0};
    HAL_StatusTypeDef status;
    uint8_t index;

    if (Number == 0 || Number > 4)
    {
        return HAL_ERROR;
    }

    index = Number - 1;
    HAL_GPIO_WritePin(cs[index].port, cs[index].pin, GPIO_PIN_RESET);
    Encoder_DelayUs(1);                       /* CSN 建立时间 */
    status = HAL_SPI_TransmitReceive(hspi4, (uint8_t *)tx, rx, 4, 10);
    HAL_GPIO_WritePin(cs[index].port, cs[index].pin, GPIO_PIN_SET);
    Encoder_DelayUs(2);                       /* 帧间隔，CSN 保持高电平 */

    return status;
}

/**
 * @brief CRC6 校验：多项式 x^6+x+1，MSB 先行，初值 0
 * @param data 待校验数据（最高位先移入）
 * @param len  数据位数
 * @return CRC6
 */
static uint8_t crc6_calc(uint32_t data, uint32_t len)
{
    uint8_t crc = 0;

    for (int32_t i = (int32_t)len - 1; i >= 0; i--)
    {
        uint8_t top = (uint8_t)((crc >> 5) & 0x01U);

        crc = (uint8_t)(((crc << 1) | ((data >> i) & 0x01U)) & 0x3FU);

        if (top != 0U)
        {
            crc ^= 0x03U;
        }
    }

    return crc;
}

/**
 * @brief 4 字节原始数据 -> 24bit 有效帧（两种对齐都试，整帧 CRC 余数为 0 才算有效）
 * @return 1 有效，0 无效
 */
static uint8_t Frame_To_Valid24(const uint8_t rx[4], uint32_t *valid24)
{
    uint32_t frame32;
    uint32_t v;

    frame32 = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) |
              ((uint32_t)rx[2] << 8) | (uint32_t)rx[3];

    v = ((frame32 << 1) >> 8) & 0xFFFFFFU;  // 丢掉首位无效位 
    if (crc6_calc(v, 24U) == 0U)
    {
        *valid24 = v;
        return 1U;
    }

    v = (frame32 >> 8) & 0xFFFFFFU;  // 首位就是有效数据 
    if (crc6_calc(v, 24U) == 0U)
    {
        *valid24 = v;
        return 1U;
    }

    return 0U;
}

/**
 * @brief 读一帧并解析出角度原始值、磁场状态、CRC
 * @return 1 表示读到有效帧，0 表示失败
 */
static uint8_t Encoder_ReadAngle(uint16_t *angle, uint8_t *mag, uint8_t *crc_rx,
                                 uint32_t *frame24, uint8_t Number)
{
    uint8_t rx[4];
    uint32_t valid24;

    if (Encoder_Read(Number, rx) != HAL_OK)
    {
        return 0;
    }

    if (Frame_To_Valid24(rx, &valid24) == 0U)
    {
        return 0;
    }

    *frame24 = valid24;
    *angle = (uint16_t)((valid24 >> 10) & 0x3FFFU);
    *mag = (uint8_t)((valid24 >> 6) & 0x0FU);
    *crc_rx = (uint8_t)(valid24 & 0x3FU);
    return 1;
}

/**
 * @brief 编码器所有功能
 * @param Number 1~4
 * @return 0~360 度；返回 -1 表示读取/校验失败(不能当成 0 度用)
 */
float Abs_Encoder_Function(uint8_t Number)
{
    uint16_t raw14_rx;
    uint8_t mag_rx;
    uint32_t frame24;
    int32_t cdeg_frame;
    uint8_t crc_rx_frame;
    float cdeg;

    if (Encoder_ReadAngle(&raw14_rx, &mag_rx, &crc_rx_frame, &frame24, Number) == 0)
    {
        return -1.0f;                         /* SPI 读失败 / CRC 校验失败 */
    }

    /* 14bit 原始角度 -> 0.01 度：36000 = 360 * 100，16384 = 2^14 */
    cdeg_frame = (int32_t)((uint32_t)raw14_rx * 36000 / 16384);

    cdeg = (float)cdeg_frame / 100.0f;
    return cdeg;
}
