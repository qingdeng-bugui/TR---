
/************************* 头文件 *************************/

#include "Wave.h"

/************************ 变量定义 ************************/

/* 一次采样的两个量，单位：毫转/秒（转/秒 × 1000），int16 够用（±32.7 转/秒） */
typedef struct
{
    int16_t actual_milli;   /* 当前速度（实测反馈） */
    int16_t target_milli;   /* 目标速度（斜坡后的给定，就是 PID 跟踪的那个值） */
} WaveSample_t;

static WaveSample_t     s_buf[WAVE_BUF_LEN];
static volatile uint16_t s_head;   /* 写指针，只有 TIM6 中断改 */
static volatile uint16_t s_tail;   /* 读指针，只有主循环改 */

/************************ 函数定义 ************************/

/************************************************************
 * 名称   :     Wave_ToMilli
 * 功能 	:     转/秒 → 毫转/秒的整数（四舍五入并限幅）
 * 参数  	:     rps  转速，单位 转/秒
 * 返回值 :     毫转/秒
 ************************************************************/

static int16_t Wave_ToMilli(float rps)
{
    float v = rps * 1000.0f;

    if (v > 32000.0f)
    {
        v = 32000.0f;
    }
    if (v < -32000.0f)
    {
        v = -32000.0f;
    }

    return (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/************************************************************
 * 名称   :     Wave_AppendU32
 * 功能 	:     把无符号整数写成十进制字符串（不依赖 printf）
 * 参数  	:     p  写指针
 *              v  数值
 * 返回值 :     写完后的新写指针
 ************************************************************/

static char *Wave_AppendU32(char *p, uint32_t v)
{
    char    tmp[10];
    uint8_t n = 0U;

    do
    {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v != 0U);

    while (n > 0U)
    {
        *p++ = tmp[--n];
    }

    return p;
}

/************************************************************
 * 名称   :     Wave_AppendMilli
 * 功能 	:     把毫转/秒写成 "±整数.三位小数"，例如 -0.800
 * 参数  	:     p      写指针
 *              milli  毫转/秒
 * 返回值 :     写完后的新写指针
 * 说明  	:     自己拼字符串是为了绕开 printf 的 %f —— newlib-nano 下
 *              %f 要额外链接 _printf_float，而且会带进来十几 KB 的库。
 ************************************************************/

static char *Wave_AppendMilli(char *p, int32_t milli)
{
    uint32_t frac;

    if (milli < 0)
    {
        *p++ = '-';
        milli = -milli;
    }

    p = Wave_AppendU32(p, (uint32_t)(milli / 1000));
    *p++ = '.';

    frac  = (uint32_t)(milli % 1000);
    *p++ = (char)('0' + (frac / 100U));         /* 十分位 */
    *p++ = (char)('0' + ((frac / 10U) % 10U));  /* 百分位 */
    *p++ = (char)('0' + (frac % 10U));          /* 千分位 */

    return p;
}

/************************************************************
 * 名称   :     Wave_Init
 * 功能 	:     清空环形缓冲
 * 参数  	:     无
 * 返回值 :     无
 * 说明  	:     必须在 HAL_TIM_Base_Start_IT(&htim6) 之前调用
 ************************************************************/

void Wave_Init(void)
{
    s_head = 0U;
    s_tail = 0U;
}

/************************************************************
 * 名称   :     Wave_Sample
 * 功能 	:     采一次"当前速度 + 目标速度"入队
 * 参数  	:     无
 * 返回值 :     无
 * 说明  	:     在 TIM6 的 5ms 中断里调用。缓冲满了就丢这一拍，
 *              绝不在中断里等待，保证控制周期不被拖慢。
 ************************************************************/

void Wave_Sample(void)
{
    uint16_t next = (uint16_t)((s_head + 1U) % WAVE_BUF_LEN);

    if (next == s_tail)
    {
        return;                                 /* 满了，丢这一拍 */
    }

    s_buf[s_head].actual_milli = Wave_ToMilli(g_motor[WAVE_MOTOR_ID].wheel_rps);
    s_buf[s_head].target_milli = Wave_ToMilli(Motor_GetSpeed(WAVE_MOTOR_ID));
    s_head = next;
}

/************************************************************
 * 名称   :     Wave_Send
 * 功能 	:     把队列里最老的一拍发出去（一行两列，CRLF 结尾）
 * 参数  	:     无
 * 返回值 :     无
 * 说明  	:     在主循环里调用，一次只发一行，发不完下一轮继续发。
 *              串口比采样慢的时候会自动积压，积压超过缓冲深度才开始丢点。
 ************************************************************/

void Wave_Send(void)
{
    char line[20];
    char *p = line;

    if (s_tail == s_head)
    {
        return;                                 /* 没数据 */
    }

    p = Wave_AppendMilli(p, s_buf[s_tail].actual_milli);
    *p++ = ',';
    p = Wave_AppendMilli(p, s_buf[s_tail].target_milli);
    *p++ = '\r';
    *p++ = '\n';

    (void)HAL_UART_Transmit(&huart8, (uint8_t *)line, (uint16_t)(p - line), 5U);

    s_tail = (uint16_t)((s_tail + 1U) % WAVE_BUF_LEN);
}

/****************************End*****************************/
