#include "self_controller.h"
#include "bsp_usart.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "memory.h"
#include "usart.h"
#include "robot_def.h"

#include "seasky_protocol.h"
#include "crc8.h"
#include "crc16.h"
#include "stdint.h"
#include "crc_ref.h"
#include "daemon.h"
#include "cmsis_os.h"

static USARTInstance *customize;
static DaemonInstance *customize_daemon;
static customize_info_t customize_info;
static Self_Cntlr_s self_cntlr;
static uint8_t data[30];

// 用于检测图传数据频率
#define FreqDetect()                                                   \
    {                                                                  \
        static TickType_t time_start, time_now, count;                 \
        static float freq;                                             \
                                                                       \
        count++;                                                       \
        if (count == 1)                                                \
            time_start = xTaskGetTickCount();                          \
        time_now = xTaskGetTickCount();                                \
        if (time_now - time_start >= 1000)                             \
        {                                                              \
            freq = count / (float)((time_now - time_start) / 1000.0f); \
            count = 0;                                                 \
        }                                                              \
    }

static void customizeReadData(uint8_t *buff)
{
    uint16_t customizedata; // 统计一帧数据长度
    if (buff == NULL)       // 空数据包，则不作任何处理
        return;

    // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
    memcpy(&customize_info.FrameHeader, buff, 5);

    // 判断帧头数据(0)是否为0xA5
    if (buff[SOF_customize] == 0xA5)
    {
        // 帧头CRC8校验
        if (Verify_CRC8_Check_Sum(buff, 5) == TRUE)
        {
            // 统计一帧数据长度(byte),用于CR16校验
            customizedata = buff[DATA_LENGTH_customize] + 9;
            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(buff, customizedata) == TRUE)
            {
                // 2个8位拼成16位int
                customize_info.ID = (buff[6] << 8 | buff[5]);
                // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                // 第8个字节开始才是数据 data=7
                switch (customize_info.ID)
                {
                case 0x0302: // 0x0001
                    // FreqDetect();
                    memcpy(&data, (buff + 7), 30);
                    break;
                }
            }
        }
        // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
    }
}

static void Hex_Float(const uint8_t *hdata, float *fdata)
{
    // 假设 float 占用 4 个字节
    if (hdata != NULL && fdata != NULL)
    {
        memcpy(fdata, hdata, sizeof(float));
    }
}

static void RefereeRxCallback()
{
    // FreqDetect();
    DaemonReload(customize_daemon);
    customizeReadData(customize->recv_buff);

    Hex_Float(data + 0, &self_cntlr.roll);
    Hex_Float(data + 4, &self_cntlr.yaw);
    Hex_Float(data + 8, &self_cntlr.pitch);
    Hex_Float(data + 12, &self_cntlr.push_dist);
    Hex_Float(data + 16, &self_cntlr.traverse_dist);
}

static void RefereeLostCallback(void *arg)
{
    USARTServiceInit(customize);
    LOGWARNING("[rm_ref] lost referee data");
}

Self_Cntlr_s *SelfCntlrInit(UART_HandleTypeDef *sc_usart_handle)
{
    USART_Init_Config_s custom_config = {
        .usart_handle = sc_usart_handle,
        .recv_buff_size = 39,
        .module_callback = RefereeRxCallback,
    };
    customize = USARTRegister(&custom_config);
    Daemon_Init_Config_s daemon_conf = {
        .callback = RefereeLostCallback,
        .owner_id = customize,
        .reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
    };
    customize_daemon = DaemonRegister(&daemon_conf);

    return &self_cntlr;
}
