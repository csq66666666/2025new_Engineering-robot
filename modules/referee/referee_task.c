/**
 * @file referee.C
 * @author liyong@stu.cdu.edu.cn
 * @brief
 * @version final
 * @date 2024-05-29
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
    MyUIRefresh(referee_recv_info, Interactive_data);
}

static String_Data_t UI_State_sta[2]; // 机器人模式状态,静态只需画一次 静态(始终存在画面)
static String_Data_t UI_State_dyn[2]; // 机器人模式状态,动态先add才能change（始终存在画面）
static String_Data_t UI_Warning[3];   // 警告信息
static Graph_Data_t UI_Pump[5];       // 泵和阀的状态

void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制车辆状态标志指示，静态
    UICharDraw(&UI_State_sta[0], "ss1", UI_Graph_ADD, 8, UI_Color_Orange, 25, 5, 40, 750, "CHASSIS:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_sta[1], "ss2", UI_Graph_ADD, 8, UI_Color_Orange, 25, 5, 40, 650, "UPPER:  ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_ADD, 8, UI_Color_Purplish_red, 25, 5, 100, 700, "          ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_ADD, 8, UI_Color_Purplish_red, 25, 5, 100, 600, "          ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);

    UICharDraw(&UI_Warning[0], "wn1", UI_Graph_ADD, 8, UI_Color_Green, 50, 5, 680, 900, "           ");
    UICharRefresh(&referee_recv_info->referee_id, UI_Warning[0]);
    UICharDraw(&UI_Warning[1], "wn2", UI_Graph_ADD, 8, UI_Color_Green, 50, 5, 680, 800, "           ");
    UICharRefresh(&referee_recv_info->referee_id, UI_Warning[1]);
    UICharDraw(&UI_Warning[2], "wn3", UI_Graph_ADD, 8, UI_Color_Green, 50, 5, 680, 700, "           ");
    UICharRefresh(&referee_recv_info->referee_id, UI_Warning[2]);

    // 气泵和电磁阀，动态
    UICircleDraw(&UI_Pump[0], "pmp", UI_Graph_ADD, 7, UI_Color_Main, 10, 300, 850, 25);
    UICircleDraw(&UI_Pump[1], "vl1", UI_Graph_ADD, 7, UI_Color_Main, 10, 960, 250, 25);
    UICircleDraw(&UI_Pump[2], "vl2", UI_Graph_ADD, 7, UI_Color_Main, 10, 800, 150, 25);
    UICircleDraw(&UI_Pump[3], "vl3", UI_Graph_ADD, 7, UI_Color_Main, 10, 960, 150, 25);
    UICircleDraw(&UI_Pump[4], "vl4", UI_Graph_ADD, 7, UI_Color_Main, 10, 1120, 150, 25);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_Pump[0], UI_Pump[1], UI_Pump[2], UI_Pump[3], UI_Pump[4]);
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // 模式切换显示部分
    //  PUMP & VALVE
    if (_Interactive_data->Referee_Interactive_Flag.pump_flag == 1)
    {
        if (_Interactive_data->pump_mode)
        {
            UICircleDraw(&UI_Pump[0], "pmp", UI_Graph_Change, 7, UI_Color_Green, 10, 300, 850, 25);
        }
        else
        {
            UICircleDraw(&UI_Pump[0], "pmp", UI_Graph_Change, 7, UI_Color_Main, 10, 300, 850, 25);
        }
        if (_Interactive_data->pump_mode & VAVLVE_ARM)
        {
            UICircleDraw(&UI_Pump[1], "vl1", UI_Graph_Change, 7, UI_Color_Green, 10, 960, 250, 25);
        }
        else
        {
            UICircleDraw(&UI_Pump[1], "vl1", UI_Graph_Change, 7, UI_Color_Main, 10, 960, 250, 25);
        }
        if (_Interactive_data->pump_mode & VAVLVE_T1)
        {
            UICircleDraw(&UI_Pump[2], "vl2", UI_Graph_Change, 7, UI_Color_Green, 10, 800, 150, 25);
        }
        else
        {
            UICircleDraw(&UI_Pump[2], "vl2", UI_Graph_Change, 7, UI_Color_Main, 10, 800, 150, 25);
        }
        if (_Interactive_data->pump_mode & VAVLVE_T2)
        {
            UICircleDraw(&UI_Pump[3], "vl3", UI_Graph_Change, 7, UI_Color_Green, 10, 960, 150, 25);
        }
        else
        {
            UICircleDraw(&UI_Pump[3], "vl3", UI_Graph_Change, 7, UI_Color_Main, 10, 960, 150, 25);
        }
        if (_Interactive_data->pump_mode & VAVLVE_T3)
        {
            UICircleDraw(&UI_Pump[4], "vl4", UI_Graph_Change, 7, UI_Color_Green, 10, 1120, 150, 25);
        }
        else
        {
            UICircleDraw(&UI_Pump[4], "vl4", UI_Graph_Change, 7, UI_Color_Main, 10, 1120, 150, 25);
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_Pump[0], UI_Pump[1], UI_Pump[2], UI_Pump[3], UI_Pump[4]);
        _Interactive_data->Referee_Interactive_Flag.pump_flag = 0;
    }
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        switch (_Interactive_data->chassis_mode)
        {
        // 此处注意字数对齐问题，字数相同才能覆盖掉
        case CHASSIS_ZERO_FORCE:
            UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_Change, 8, UI_Color_Purplish_red, 25, 5, 100, 700, "ZERO_FORCE");
            UICharDraw(&UI_Warning[0], "wn1", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 900, "WARNING:CHASSIS ZERO FORCE");
            break;
        case CHASSIS_NORMAL:
            UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_Change, 8, UI_Color_Green, 25, 5, 100, 700, "NORMAL    ");
            UICharDraw(&UI_Warning[0], "wn1", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 900, "                           ");
            break;
        case CHASSIS_MINING:
            UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_Change, 8, UI_Color_Green, 25, 5, 100, 700, "MINING    ");
            UICharDraw(&UI_Warning[0], "wn1", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 900, "                           ");
            break;
        case CHASSIS_CHARGE:
            UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_Change, 8, UI_Color_Green, 25, 5, 100, 700, "CHARGE    ");
            UICharDraw(&UI_Warning[0], "wn1", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 900, "                           ");
            break;
        case CHASSIS_NO_MOVE:
            UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 25, 5, 100, 700, "LOCKED    ");
            UICharDraw(&UI_Warning[0], "wn1", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 900, "  WARNING:CHASSIS LOCKED  ");
            break;
        }

        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        UICharRefresh(&referee_recv_info->referee_id, UI_Warning[0]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }
    // upper
    if (_Interactive_data->Referee_Interactive_Flag.upper_flag == 1)
    {
        switch (_Interactive_data->upper_mode)
        {
        // 此处注意字数对齐问题，字数相同才能覆盖掉
        case UPPER_ZERO_FORCE:
            UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_Change, 8, UI_Color_Purplish_red, 25, 5, 100, 600, "ZERO_FORCE   ");
            UICharDraw(&UI_Warning[1], "wn2", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 800, "WARNING:UPPER ZERO FORCE ");
            break;
        case UPPER_NO_MOVE:
            UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_Change, 8, UI_Color_Yellow, 25, 5, 100, 600, "NO_MOVE      ");
            UICharDraw(&UI_Warning[1], "wn2", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 800, "  WARNING:UPPER NO MOVE  ");
            break;
        case UPPER_CALI:
            UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_Change, 8, UI_Color_Yellow, 25, 5, 100, 600, "CALI         ");
            UICharDraw(&UI_Warning[1], "wn2", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 800, "WARNING:UPPER IS CAILING ");
            break;
        case UPPER_SINGLE_MOTOR:
            UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_Change, 8, UI_Color_Green, 25, 5, 100, 600, "SINGLE_MOTOR ");
            UICharDraw(&UI_Warning[1], "wn2", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 800, "                          ");
            break;
        case UPPER_SLIVER_MINING:
            UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_Change, 8, UI_Color_Green, 25, 5, 100, 600, "SINGLE_SLIVER");
            UICharDraw(&UI_Warning[1], "wn2", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 800, "WARNING:GET SINGLE SLIVER");
            break;
        case UPPER_THREE_SLIVER_MINING:
            UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_Change, 8, UI_Color_Green, 25, 5, 100, 600, "THREE_SLIVER ");
            UICharDraw(&UI_Warning[1], "wn2", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 800, "WARNING:GET THREE SLIVER ");
            break;
        case UPPER_GLOD_MINING:
            UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_Change, 8, UI_Color_Green, 25, 5, 100, 600, "GET_GLOD     ");
            UICharDraw(&UI_Warning[1], "wn2", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 800, " WARNING:GET SINGLE GLOD ");
            break;
        case UPPER_GROUND_MINING:
            UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_Change, 8, UI_Color_Green, 25, 5, 100, 600, "GET_GROUND   ");
            UICharDraw(&UI_Warning[1], "wn2", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 800, "   WARNING:GET GROUND    ");
            break;
        case UPPER_EXCHANGE:
            UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_Change, 8, UI_Color_Green, 25, 5, 100, 600, "EXCHANGE     ");
            UICharDraw(&UI_Warning[1], "wn2", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 450, 800, "   WARNING:EXCHANGING    ");
            break;
        }

        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        UICharRefresh(&referee_recv_info->referee_id, UI_Warning[1]);
        _Interactive_data->Referee_Interactive_Flag.upper_flag = 0;
    }
    // GIMBAL
    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
    {
        switch (_Interactive_data->gimbal_mode)
        {
        case Steering_gear_door_open:
            UICharDraw(&UI_Warning[2], "wn3", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 680, 700, "THE DOOR IS OPEN!");
            break;
        case Steering_gear_door_close:
            UICharDraw(&UI_Warning[2], "wn3", UI_Graph_Change, 8, UI_Color_Purplish_red, 50, 5, 680, 700, "                 ");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_Warning[2]);
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->upper_mode != _Interactive_data->upper_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.upper_flag = 1;
        _Interactive_data->upper_last_mode = _Interactive_data->upper_mode;
    }

    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }

    if (_Interactive_data->pump_mode != _Interactive_data->pump_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.pump_flag = 1;
        _Interactive_data->pump_last_mode = _Interactive_data->pump_mode;
    }
}
