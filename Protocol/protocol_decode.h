#ifndef __PROTOCOL_DECODE_H
#define __PROTOCOL_DECODE_H

/************************* 头文件 *************************/

#include "HeaderFiles.h"

/************************* 宏定义 *************************/

#define PROTO_FRAME_LEN     15U         // 一帧 15 字节
#define PROTO_HEAD          0xAAU       // 帧头

/*
 * 帧格式（和 F103 发送端/接收端完全一致，小端）：
 *   [0]     0xAA
 *   [1]     seq          序号，0~255 循环
 *   [2..3]  lx           左摇杆 X   -1000~+1000（左负右正）
 *   [4..5]  ly           左摇杆 Y   -1000~+1000（下负上正）
 *   [6..7]  rx           右摇杆 X   -1000~+1000（左负右正）
 *   [8..9]  ry           右摇杆 Y   -1000~+1000（下负上正）
 *   [10..11] enc1        编码器 1   每格 ±2
 *   [12]    keys         按键位图，bit0~bit7 = KEY_1~KEY_8
 *   [13]    flags        拨杆，bit0 = MODE1，bit1 = MODE2
 *   [14]    sum          前 14 字节求和取低 8 位
 *
 * 用法：
 *   Protocol_Init();                                    上电调一次
 *   串口每收到 1 字节 → Protocol_Feed(byte, HAL_GetTick());
 *                        返回 1 = 刚收齐一帧且校验通过
 *   主循环里定期调     → Protocol_Tick(HAL_GetTick(), 200);
 *                        200ms 没有效帧 → Protocol_GetLinkOk() 变 0
 */

/************************ 变量定义 ************************/

/************************ 函数定义 ************************/

void Protocol_Init(void);                                       // 解析器初始化

uint8_t Protocol_Feed(uint8_t byte, uint32_t now_ms);           // 喂一个字节，1=收齐一帧

void Protocol_Tick(uint32_t now_ms, uint32_t timeout_ms);       // 超时检查（失联保护）

/* ---- 字段取值（Protocol_Feed 返回 1 之后才有效） ---- */

int16_t Protocol_GetLx(void);                                   // 左摇杆 X
int16_t Protocol_GetLy(void);                                   // 左摇杆 Y
int16_t Protocol_GetRx(void);                                   // 右摇杆 X
int16_t Protocol_GetRy(void);                                   // 右摇杆 Y
int16_t Protocol_GetEnc1(void);                                 // 编码器 1
uint8_t Protocol_GetKeys(void);                                 // 按键位图
uint8_t Protocol_GetFlags(void);                                // 拨杆 bit0/bit1

/* ---- 链路状态 ---- */

uint8_t Protocol_GetLinkOk(void);                               // 1=链路正常 0=失联

#endif

/****************************End*****************************/
