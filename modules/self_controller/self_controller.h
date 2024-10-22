#ifndef SELF_CONTROLLER_H
#define SELF_CONTROLLER_H

#include "usart.h"
#include "referee_protocol.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

#pragma pack(1)

typedef struct
{
    uint8_t SOF_customize;
    uint16_t DataLength_customize;
    uint8_t Seq_customize;
    uint8_t CRC8_customize;
} Header;

typedef struct
{
    uint16_t ID;
    Header FrameHeader; // 接收到的帧头信息
} customize_info_t;

typedef enum
{
    SOF_customize = 0,         // 起始位
    DATA_LENGTH_customize = 1, // 帧内数据长度,根据这个来获取数据长度
    SEQ_customize = 3,         // 包序号
    CRC8_customize = 4         // CRC8
} FrameHeaderOffset_e_customize;

typedef struct
{
    float pitch;
    float yaw;
    float roll;
    float lift_dist;
    float push_dist;
    float traverse_dist;
} Self_Cntlr_s;

#pragma pack()

Self_Cntlr_s *SelfCntlrInit(UART_HandleTypeDef *sc_usart_handle);
// void Customizetask();

#endif