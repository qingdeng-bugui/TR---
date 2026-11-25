
/************************* 头文件 *************************/

#include "protocol_decode.h"

/************************* 宏定义 *************************/

/************************ 变量定义 ************************/

/* 解析器状态全部放静态变量，对外只暴露函数 */
static uint8_t  s_buf[PROTO_FRAME_LEN];   // 攒帧缓冲
static uint8_t  s_idx;                    // 当前收到第几字节
static uint8_t  s_last_seq;               // 上一帧序号
static uint8_t  s_has_prev;               // 是否收到过至少一帧
static uint32_t s_tick_ok;                // 最近一次收帧时刻(ms)
static uint8_t  s_link_ok;                // 1 = 链路正常
static uint32_t s_cnt_ok;                 // 校验通过帧数
static uint32_t s_cnt_bad;                // 校验失败次数
static uint32_t s_cnt_lost;               // 估算丢帧数

/************************ 函数定义 ************************/

/************************************************************
 * 名称   :     GetI16
 * 功能 	:     从缓冲区取一个 int16（小端）
 * 参数  	:     b   缓冲区
 *              idx 起始下标
 * 返回值 :     取出的 int16
 ************************************************************/

static int16_t GetI16(const uint8_t *b, uint8_t idx)
{
    return (int16_t)(((uint16_t)b[idx]) | ((uint16_t)b[idx + 1] << 8));
}

/************************************************************
 * 名称   :     Sum14
 * 功能 	:     前 14 字节求和取低 8 位（校验用）
 * 参数  	:     b   缓冲区
 * 返回值 :     校验和
 ************************************************************/

static uint8_t Sum14(const uint8_t *b)
{
    uint8_t sum = 0;
    uint8_t i;

    for (i = 0; i < 14U; i++)
    {
        sum = (uint8_t)(sum + b[i]);
    }
    return sum;
}

/************************************************************
 * 名称   :     Protocol_Init
 * 功能 	:     初始化帧解析器
 * 参数  	:     无
 * 返回值 :     无
 ************************************************************/

void Protocol_Init(void)
{
    uint8_t i;

    for (i = 0; i < PROTO_FRAME_LEN; i++)
    {
        s_buf[i] = 0;
    }
    s_idx      = 0;
    s_last_seq = 0;
    s_has_prev = 0;
    s_tick_ok  = 0;
    s_link_ok  = 0;
    s_cnt_ok   = 0;
    s_cnt_bad  = 0;
    s_cnt_lost = 0;
}

/************************************************************
 * 名称   :     Protocol_Feed
 * 功能 	:     喂一个字节进来做解码
 * 参数  	:     byte   刚从串口收到的字节
 *              now_ms 当前毫秒（HAL_GetTick()），用于失联判断
 * 返回值 :     1 = 刚收齐一帧且校验通过（字段取值函数此刻有效）
 *              0 = 还没收齐 / 校验失败
 * 说明   :     不用等帧头再收——每次都对 15 字节窗口做校验，不对就把窗口
 *              左移一格再试。从半帧中间开始接收最多丢 15 字节就能重新同步。
 ************************************************************/

uint8_t Protocol_Feed(uint8_t byte, uint32_t now_ms)
{
    uint8_t i;

    /* 还没开始攒帧：等帧头 */
    if (s_idx == 0U && byte != PROTO_HEAD)
    {
        return 0;
    }

    s_buf[s_idx] = byte;
    s_idx++;

    if (s_idx < PROTO_FRAME_LEN)
    {
        return 0;                       /* 还没收齐 */
    }

    /* 收齐 15 字节：校验 */
    if (Sum14(s_buf) == s_buf[14])
    {
        /* 序号连续性：正常应该 +1（允许 0~255 回绕） */
        if (s_has_prev)
        {
            uint8_t expect = (uint8_t)(s_last_seq + 1U);
            uint8_t diff   = (uint8_t)(s_buf[1] - expect);

            s_cnt_lost += (uint32_t)diff;
        }
        s_last_seq = s_buf[1];
        s_has_prev = 1;
        s_cnt_ok++;
        s_tick_ok = now_ms;
        s_link_ok = 1;
        s_idx     = 0;                  /* 准备下一帧 */

        return 1;
    }

    /* 校验失败：窗口左移一格继续找（不清零，避免半帧死等） */
    s_cnt_bad++;
    for (i = 1; i < PROTO_FRAME_LEN; i++)
    {
        s_buf[i - 1U] = s_buf[i];
    }
    s_idx = (uint8_t)(PROTO_FRAME_LEN - 1U);

    return 0;
}

/************************************************************
 * 名称   :     Protocol_Tick
 * 功能 	:     超时检查（失联保护）
 * 参数  	:     now_ms      当前毫秒
 *              timeout_ms  超过这么久没收到有效帧就算失联
 * 返回值 :     无
 * 说明   :     Protocol_GetLinkOk() 变 0 后应该做安全动作（电机停、舵机回中）
 ************************************************************/

void Protocol_Tick(uint32_t now_ms, uint32_t timeout_ms)
{
    if (s_cnt_ok == 0U)
    {
        s_link_ok = 0;
        return;
    }

    if ((uint32_t)(now_ms - s_tick_ok) > timeout_ms)
    {
        s_link_ok = 0;
    }
    else
    {
        s_link_ok = 1;
    }
}

/************************************************************
 * 名称   :     Protocol_GetLx / Ly / Rx / Ry / Enc1
 * 功能 	:     取摇杆和编码器值
 * 参数  	:     无
 * 返回值 :     对应字段（int16）
 ************************************************************/

int16_t Protocol_GetLx(void)    { return GetI16(s_buf, 2);  }
int16_t Protocol_GetLy(void)    { return GetI16(s_buf, 4);  }
int16_t Protocol_GetRx(void)    { return GetI16(s_buf, 6);  }
int16_t Protocol_GetRy(void)    { return GetI16(s_buf, 8);  }
int16_t Protocol_GetEnc1(void)  { return GetI16(s_buf, 10); }

/************************************************************
 * 名称   :     Protocol_GetKeys / GetFlags / GetSeq
 * 功能 	:     取按键位图、拨杆状态、帧序号
 * 参数  	:     无
 * 返回值 :     对应字段（uint8）
 ************************************************************/

uint8_t Protocol_GetKeys(void)  { return s_buf[12]; }
uint8_t Protocol_GetFlags(void) { return s_buf[13]; }
uint8_t Protocol_GetSeq(void)   { return s_buf[1];  }

/************************************************************
 * 名称   :     Protocol_GetLinkOk / GetOkCount / GetBadCount / GetLostCount
 * 功能 	:     取链路状态和调试计数
 * 参数  	:     无
 * 返回值 :     对应字段
 ************************************************************/

uint8_t  Protocol_GetLinkOk(void)   { return s_link_ok;   }
uint32_t Protocol_GetOkCount(void)  { return s_cnt_ok;    }
uint32_t Protocol_GetBadCount(void) { return s_cnt_bad;   }
uint32_t Protocol_GetLostCount(void){ return s_cnt_lost;  }

/****************************End*****************************/
