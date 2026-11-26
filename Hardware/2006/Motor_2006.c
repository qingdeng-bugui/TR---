#include "Motor_2006.h" //2006电机
#include "can.h"
#include "PID_2006.h"

/* 0x200 控制帧缓存: DATA[0..1]~DATA[6..7] 分别对应 ID1~ID4 的电流值 */
static uint8_t Motor_Current_Data[8] = {0};

/* 收到的有效反馈帧累计数(0x201~0x204), 用来判断反馈是否丢失 */
volatile uint32_t Motor_2006_RxCount = 0;

/**
 * @brief 2006电机初始化
 * @retval void
 */
void Motor_2006_Init(void)
{
    HAL_CAN_Start(&hcan2); //启动CAN2 

    CAN_FilterTypeDef filterInit;
    filterInit.FilterIdHigh = 0x200 << 5; //设置过滤器ID为0x200
    filterInit.FilterIdLow = 0x0000;
    filterInit.FilterMaskIdHigh = 0x7f8 << 5; //掩码0x7f8: 同时接收 0x200 与 0x201~0x204 反馈帧
    filterInit.FilterMaskIdLow = 0x0000;
    filterInit.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filterInit.FilterBank = 14; //CAN2 使用 bank14~27
    filterInit.FilterMode = CAN_FILTERMODE_IDMASK;
    filterInit.FilterScale = CAN_FILTERSCALE_32BIT;
    filterInit.FilterActivation = CAN_FILTER_ENABLE;
    filterInit.SlaveStartFilterBank = 14; 
    HAL_CAN_ConfigFilter(&hcan2, &filterInit); //配置CAN2过滤器

    PID_2006_Init(&g_pid[0]);
    PID_2006_Init(&g_pid[1]);
    PID_2006_Init(&g_pid[2]);
    PID_2006_Init(&g_pid[3]);
    
    Abs_Zero_Init();
}

/**
 * @brief 2006电机发送数据
 * @param hcanx CAN句柄
 * @param id CAN ID
 * @param data 数据
 * @retval 发送状态
 */
static uint8_t Motor_2006_Send(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    txHeader.StdId = id; //设置标准ID
    txHeader.ExtId = 0; //扩展ID为0
    txHeader.IDE = CAN_ID_STD; //使用标准ID
    txHeader.RTR = CAN_RTR_DATA; //数据帧
    txHeader.DLC = 8; //数据长度为8字节

    return HAL_CAN_AddTxMessage(hcan, &txHeader, data, &txMailbox); //发送CAN消息
    

}


/**
 * @brief 2006电机设置速度
 * @param Current，电机电流值，范围-10000~10000，单位mA
 * @note 发送的数据格式为：data[x] = Current高8位，data[x+1] = Current低8位
 * @note CAN ID为0x200
 * @param Number，电机编号，范围1~4
 * @retval 发送状态
 */
uint8_t Motor_2006_SetSpeed(int16_t Current,uint8_t Number)
{
    uint8_t status;
    if (Number < 1 || Number > 4)
    {
        return 0;
    }
    if (Current > 10000)
    {
        Current = 10000;
    }
    else if (Current < -10000)
    {
        Current = -10000;
    }
    //只更新自己那一路, 其余三路保持上一次下发的值
    Motor_Current_Data[(Number - 1) * 2]     = (uint8_t)((Current >> 8) & 0xFF);
    Motor_Current_Data[(Number - 1) * 2 + 1] = (uint8_t)(Current & 0xFF);
    status = Motor_2006_Send(&hcan2, 0x200, Motor_Current_Data);
    if (status != HAL_OK)
    {
        return 0;
    }
    return 1;
}

/**
 * @brief 2006电机一次性设置 4 路电流
 * @param Current1~Current4: ID1~ID4 的电流值, 范围-10000~10000, 对应电调输出 -10~10A
 * @note 4 路共用一帧 0x200, 每个控制周期只发一次, 4 个电机同时生效
 * @retval 发送状态, 1 成功 0 失败
 */
uint8_t Motor_2006_SetAll(int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4)
{
    int16_t Current[4];
    uint8_t i;
    uint8_t status;

    Current[0] = Current1;
    Current[1] = Current2;
    Current[2] = Current3;
    Current[3] = Current4;

    for (i = 0; i < 4; i++)
    {
        if (Current[i] > 10000)
        {
            Current[i] = 10000;
        }
        else if (Current[i] < -10000)
        {
            Current[i] = -10000;
        }
        Motor_Current_Data[i * 2] = (uint8_t)((Current[i] >> 8) & 0xFF);
        Motor_Current_Data[i * 2 + 1] = (uint8_t)(Current[i] & 0xFF);
    }

    status = Motor_2006_Send(&hcan2, 0x200, Motor_Current_Data);
    if (status != HAL_OK)
    {
        return 0;
    }
    return 1;
}

/**
 * @brief 2006电机获取角度
 * @note 接收的数据格式为：data[0] = Angle高8位，data[1] = Angle低8位
 * @note CAN ID为0x201
 * @param Number，电机编号，范围1~4
 * @retval 电机角度，范围0-360，单位度
 */
float Motor_2006_GetAngle(uint8_t Number)
{
    static float angle_Last[5] = {0}; //各电机上一次的单圈角度, 下标 1~4 对应 ID1~ID4
    static float angle_New[5] = {0};  //各电机最新的单圈角度, 范围0~360度
    static int16_t Count[5] = {0};    //各电机累计圈数
    static uint8_t angle_Valid[5] = {0}; //该电机是否已经收到过反馈
    CAN_RxHeaderTypeDef rxHeader; 
    uint8_t data[8] = {0};  //CAN 接收数据缓存
    uint8_t motor;  //电机编号, 0~3 对应 ID1~ID4
    uint16_t angle_raw;

    if (Number < 1 || Number > 4)
    {
        return 0.0f;
    }
    //把FIFO里的帧读完, 按 ID 分别更新 4 个电机的角度(反馈帧 0x201~0x204)
    while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK)
        {
            break;
        }
        if ((rxHeader.StdId < 0x201) || (rxHeader.StdId > 0x204))
        {
            continue;
        }
        motor = (uint8_t)(rxHeader.StdId - 0x200); //0x201 -> 1 号电机, 0x204 -> 4 号电机
        Motor_2006_RxCount++; //有效反馈帧计数
        //每帧反馈的 DATA[0..1] 就是该电机自己的机械角度
        angle_raw = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
        angle_New[motor] = (float)angle_raw / 8191.0f * 360.0f; //将原始角度转换为0~360度

        if (angle_Valid[motor] == 0) //第一帧只记录基准, 不判圈
        {
            angle_Last[motor] = angle_New[motor];
            angle_Valid[motor] = 1;
        }
        //角度从 359 度跳到 0 度(正转)时差值为负, 从 0 度跳到 359 度(反转)时差值为正
        if (angle_New[motor] - angle_Last[motor] > 180.0f)
        {
            Count[motor]--;
        }
        else if (angle_New[motor] - angle_Last[motor] < -180.0f)
        {
            Count[motor]++;
        }
        angle_Last[motor] = angle_New[motor];
    }
    return (float)Count[Number] * 360.0f + angle_New[Number]; //返回累计角度
}

