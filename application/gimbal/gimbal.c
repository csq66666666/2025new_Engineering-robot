#include "gimbal.h"
#include "robot_def.h"
#include "message_center.h"
#include "general_def.h"
#include "servo_motor.h"
#include "stdlib.h"
#include "memory.h"
#include "can_comm.h"
// cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static ServoInstance *yaw_steering_gear, *pitch_steering_gear, *Magazine_servo_l, *Magazine_servo_r;
static CANCommInstance *gimbal_can_comm;
void GimbalInit()
{

    Servo_Init_Config_s yaw_config = {
        // 舵机安装选择的定时器及通道
        // C板有常用的7路PWM输出:TIM1-1,2,3,4 TIM8-1,2,3
        .htim = &htim1,
        .Channel = TIM_CHANNEL_1,
        // 舵机的初始化模式和类型
        .Servo_Angle_Type = Start_mode,
        .Servo_type = Servo270,
    };
    yaw_steering_gear = ServoInit(&yaw_config);

    Servo_Init_Config_s pitch_config = {
        // 舵机安装选择的定时器及通道
        // C板有常用的7路PWM输出:TIM1-1,2,3,4 TIM8-1,2,3
        .htim = &htim1,
        .Channel = TIM_CHANNEL_2,
        // 舵机的初始化模式和类型
        .Servo_Angle_Type = Free_Angle_mode,
        .Servo_type = Servo180,
    };
    pitch_steering_gear = ServoInit(&pitch_config);
    // 左边舵机弹仓
    Servo_Init_Config_s Magazine_servo_config_l = {
        // 舵机安装选择的定时器及通道
        // C板有常用的7路PWM输出:TIM1-1,2,3,4 TIM8-1,2,3
        .htim = &htim1,
        .Channel = TIM_CHANNEL_3,
        // 舵机的初始化模式和类型
        .Servo_Angle_Type = Free_Angle_mode,
        .Servo_type = Servo180,
    };
    Magazine_servo_l = ServoInit(&Magazine_servo_config_l);

    // 右边舵机弹仓
    Servo_Init_Config_s Magazine_servo_config_r = {
        // 舵机安装选择的定时器及通道
        // C板有常用的7路PWM输出:TIM1-1,2,3,4 TIM8-1,2,3
        .htim = &htim1,
        .Channel = TIM_CHANNEL_4,
        // 舵机的初始化模式和类型
        .Servo_Angle_Type = Free_Angle_mode,
        .Servo_type = Servo180,
    };
    Magazine_servo_r = ServoInit(&Magazine_servo_config_r);

#ifdef CHASSIS_BOARD

    CANComm_Init_Config_s comm_conf_gimbal = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x322,
            .rx_id = 0x323,
        },
        .recv_data_len = sizeof(Gimbal_Ctrl_Cmd_s),
        .send_data_len = sizeof(Gimbal_Upload_Data_s),
    };
    gimbal_can_comm = CANCommInit(&comm_conf_gimbal); // can comm初始化
#endif
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
#ifdef CHASSIS_BOARD
    gimbal_cmd_recv = *(Gimbal_Ctrl_Cmd_s *)CANCommGet(gimbal_can_comm);
#endif

    switch (gimbal_cmd_recv.gimbal_mode)
    {

    case GIMBAL_FREE_MODE:
        Servo_Motor_FreeAngle_Set(yaw_steering_gear, gimbal_cmd_recv.yaw);
        Servo_Motor_FreeAngle_Set(pitch_steering_gear, gimbal_cmd_recv.pitch);
        ServeoMotorControl();
        break;
    case GIMBAL_DAMN_MODE:
        Servo_Motor_FreeAngle_Set(yaw_steering_gear, gimbal_cmd_recv.yaw);
        Servo_Motor_FreeAngle_Set(pitch_steering_gear, gimbal_cmd_recv.pitch);
        ServeoMotorControl();
        break;
    case GIMBAL_NORMAL:
        Servo_Motor_FreeAngle_Set(yaw_steering_gear, gimbal_cmd_recv.yaw);
        Servo_Motor_FreeAngle_Set(pitch_steering_gear, gimbal_cmd_recv.pitch);
        ServeoMotorControl();
        break;
    case GIMBAL_MINING:
        Servo_Motor_FreeAngle_Set(yaw_steering_gear, gimbal_cmd_recv.yaw);
        Servo_Motor_FreeAngle_Set(pitch_steering_gear, gimbal_cmd_recv.pitch);
        ServeoMotorControl();
        break;
    case Steering_gear_door_open:
        Servo_Motor_FreeAngle_Set(Magazine_servo_l, 150);
        Servo_Motor_FreeAngle_Set(Magazine_servo_r, 15);
        ServeoMotorControl();
        break;
    case Steering_gear_door_close:
        Servo_Motor_FreeAngle_Set(Magazine_servo_l, 50);
        Servo_Motor_FreeAngle_Set(Magazine_servo_r, 115);
        ServeoMotorControl();
        break;
    default:
        break;
    }
    Servo_Motor_FreeAngle_Set(yaw_steering_gear, gimbal_cmd_recv.yaw_end + gimbal_cmd_recv.yaw);
    Servo_Motor_FreeAngle_Set(pitch_steering_gear, gimbal_cmd_recv.pitch_end + gimbal_cmd_recv.pitch);
    ServeoMotorControl();

    switch (gimbal_cmd_recv.damn_mode)
    {
    case DAMN_KEEP:
        Servo_Motor_FreeAngle_Set(Magazine_servo_l, 50);
        Servo_Motor_FreeAngle_Set(Magazine_servo_r, 115);
        break;

    default:

        break;
    }

#ifdef CHASSIS_BOARD
    CANCommSend(gimbal_can_comm, (void *)&gimbal_feedback_data);
#endif // CHASSIS_BOARD
}