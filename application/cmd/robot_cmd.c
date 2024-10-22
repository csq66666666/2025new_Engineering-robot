// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "upper.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "user_lib.h"
#include "dji_motor.h"
#include "self_controller.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "cmsis_os.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm, *cam_can_gimbal; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

/* static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;   */
// 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Publisher_t *upper_cmd_pub;           // 上层机构控制消息发布者
static Subscriber_t *upper_feed_sub;         // 上层机构反馈信息订阅者
static Upper_Ctrl_Cmd_s upper_cmd_send;      // 传递给上层机构的控制信息
static Upper_Upload_Data_s upper_fetch_data; // 从上层机构获取的反馈信息

static TickType_t dial_time_start; // 计时开始时的时间
static TickType_t dial_time_now;   // 当前时刻时间
static uint8_t dial_press_flag;    // 拨轮按下标志位
static upper_mode_e upper_last_mode;
static uint16_t getsliver_flag = 0;

// 这个变量目前好像没什么作用，但还是先留着，后期再更改
static Robot_Status_e robot_state; // 机器人整体工作状态

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    upper_cmd_pub = PubRegister("upper_cmd", sizeof(Upper_Ctrl_Cmd_s));
    upper_feed_sub = SubRegister("upper_feed", sizeof(Upper_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);

    CANComm_Init_Config_s comm_conf_gimbal = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x323,
            .rx_id = 0x322,
        },
        .recv_data_len = sizeof(Gimbal_Upload_Data_s),
        .send_data_len = sizeof(Gimbal_Ctrl_Cmd_s),
    };
    cam_can_gimbal = CANCommInit(&comm_conf_gimbal); // can comm初始化

#endif // GIMBAL_BOARD

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
    upper_cmd_send.cfm_flag = 0;
}

/**
 * @brief 将当前位姿更新到发送端，使后续操作在当前位姿上执行
 *
 */
static void CmdRecvUpdate()
{
    upper_cmd_send.joint_data.yaw = upper_fetch_data.joint_data.yaw;
    upper_cmd_send.joint_data.roll = upper_fetch_data.joint_data.roll;
    upper_cmd_send.joint_data.pitch = upper_fetch_data.joint_data.pitch;
    upper_cmd_send.joint_data.roll_differ = upper_fetch_data.joint_data.roll_differ;
    upper_cmd_send.joint_data.pitch_differ = upper_fetch_data.joint_data.pitch_differ;
    upper_cmd_send.joint_data.lift_dist = upper_fetch_data.joint_data.lift_dist;
    upper_cmd_send.joint_data.push_dist = upper_fetch_data.joint_data.push_dist;
    upper_cmd_send.joint_data.traverse_dist = upper_fetch_data.joint_data.traverse_dist;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    chassis_cmd_send.chassis_mode = CHASSIS_NO_MOVE;
    if (upper_cmd_send.upper_mode != UPPER_CALI)
    {
        upper_cmd_send.upper_mode = UPPER_NO_MOVE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        gimbal_cmd_send.yaw = 90;
        gimbal_cmd_send.pitch = 90;
        gimbal_cmd_send.yaw_end = 0;
        gimbal_cmd_send.pitch_end = 0;
        gimbal_cmd_send.damn_mode = DAMN_KEEP;
        if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[上]
        {
            // 控制底盘运行模式
            if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],底盘正常行进
            {
                chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            }
            else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘取矿行进模式
            {
                chassis_cmd_send.chassis_mode = CHASSIS_MINING;
            }

            // 底盘参数,系数需要调整
            chassis_cmd_send.vx = 40.0f * (float)rc_data[TEMP].rc.rocker_r_; // _水平方向
            chassis_cmd_send.vy = 40.0f * (float)rc_data[TEMP].rc.rocker_r1; // |竖直方向
            chassis_cmd_send.wz = -5.0f * (float)rc_data[TEMP].rc.rocker_l_; // ↺旋转方向 遥控器摇杆从左往右值增大,与旋转方向相反，所以取相反数
        }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中]
        {
            upper_cmd_send.upper_mode = UPPER_SINGLE_MOTOR;

            if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],滑移机构控制
            {
                upper_cmd_send.joint_data.lift_dist += 0.002 * (float)rc_data[TEMP].rc.rocker_l1;
                upper_cmd_send.joint_data.yaw += 0.001f * (float)rc_data[TEMP].rc.rocker_l_;
                upper_cmd_send.joint_data.push_dist += 0.001 * (float)rc_data[TEMP].rc.rocker_r1;
                upper_cmd_send.joint_data.traverse_dist += 0.001 * (float)rc_data[TEMP].rc.rocker_r_;
            }
            else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],机械臂控制
            {
                // 按照摇杆的输出大小进行角度增量,增益系数需调整
                upper_cmd_send.joint_data.roll += 0.001f * (float)rc_data[TEMP].rc.rocker_r_;
                upper_cmd_send.joint_data.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_r1;
                upper_cmd_send.joint_data.roll_differ += 0.001f * (float)rc_data[TEMP].rc.rocker_l_;
                upper_cmd_send.joint_data.pitch_differ += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
            }

            // 真空泵控制,拨轮向上打为负,向下为正
            if (rc_data[TEMP].rc.dial < -100) // 向上打开/关闭真空泵
            {
                chassis_cmd_send.pump_mode = VAVLVE_ARM | VAVLVE_T1 | VAVLVE_T2 | VAVLVE_T3;
            }
            else if (rc_data[TEMP].rc.dial > 100)
            {
                chassis_cmd_send.pump_mode = VAVLVE_ALL_CLOSE;
            }

            // 拨轮下拨2s机械臂初始化
            if (rc_data[TEMP].rc.dial > 100) // 拨轮下拨
            {
                if (!dial_press_flag)
                {
                    dial_press_flag = 1;
                    dial_time_start = xTaskGetTickCount();
                }

                if (dial_press_flag == 1)
                {
                    dial_time_now = xTaskGetTickCount();

                    if ((dial_time_now - dial_time_start) > 2000)
                    {
                        upper_cmd_send.upper_mode = UPPER_CALI;
                    }
                }
            }
            else
            {
                dial_press_flag = 0;
            }
        }
        else if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_up(rc_data[TEMP].rc.switch_right)) // 左侧开关状态为[下] 且 右侧开关为上
        {
            /* upper_cmd_send.upper_mode = UPPER_EXCHANGE;
            upper_cmd_send.joint_data.lift_dist += 0.002 * (float)rc_data[TEMP].rc.rocker_l1;
            upper_cmd_send.joint_data.yaw += 0.001f * (float)rc_data[TEMP].rc.rocker_l_;
            upper_cmd_send.joint_data.push_dist += 0.001 * (float)rc_data[TEMP].rc.rocker_r1;
            upper_cmd_send.joint_data.traverse_dist += 0.001 * (float)rc_data[TEMP].rc.rocker_r_; */
            __set_FAULTMASK(1);
            HAL_NVIC_SystemReset();
        }
    }
    else
    {
        if (upper_fetch_data.action_step == 0)
        {
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
        }
    }
}

/**
 * @brief 控制输入为键鼠时模式和控制量设置
 *        当某个键位存在组合键时，单独使用该键位需要添加‘!’判断
 */
static void MouseKeyControlSet()
{
    // 底盘模式
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 3)
    {
    case 0:
        chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
        if (rc_data[TEMP].key[KEY_PRESS].b)
        {
            gimbal_cmd_send.gimbal_mode = GIMBAL_NORMAL;
            gimbal_cmd_send.yaw = 3;
            gimbal_cmd_send.pitch = 90;
            gimbal_cmd_send.yaw_end = 0;
            gimbal_cmd_send.pitch_end = 0;
        }
        break;
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_MINING;
        if (rc_data[TEMP].key[KEY_PRESS].b)
        {
            gimbal_cmd_send.gimbal_mode = GIMBAL_MINING;
            gimbal_cmd_send.yaw = 90;
            gimbal_cmd_send.pitch = 90;
            gimbal_cmd_send.yaw_end = 0;
            gimbal_cmd_send.pitch_end = 0;
        }
        break;
    case 2:
        chassis_cmd_send.chassis_mode = CHASSIS_CHARGE;
        if (rc_data[TEMP].key[KEY_PRESS].b)
        {
            gimbal_cmd_send.gimbal_mode = GIMBAL_DAMN_MODE;
            gimbal_cmd_send.yaw = 265;
            gimbal_cmd_send.pitch = 145;
            gimbal_cmd_send.yaw_end = 0;
            gimbal_cmd_send.pitch_end = 0;
        }
        break;
    }

    // 底盘键鼠控制
    //  W/S // A/D // Q/E
    // 左右 // 前后 // 自旋
    if (chassis_cmd_send.chassis_mode == CHASSIS_NORMAL)
    {
        if (!(rc_data[TEMP].key[KEY_PRESS].shift) && !(rc_data[TEMP].key[KEY_PRESS].ctrl))
        // W/A/S/D 底盘移动
        {
            chassis_cmd_send.vx = 26000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].a - (float)rc_data[TEMP].key[KEY_PRESS].d); // _水平方向
            chassis_cmd_send.vy = 26000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].w - (float)rc_data[TEMP].key[KEY_PRESS].s); // |竖直方向
            chassis_cmd_send.wz = 1500.0f * ((float)rc_data[TEMP].key[KEY_PRESS].q - (float)rc_data[TEMP].key[KEY_PRESS].e);  // ↺自旋
        }
        else
        {
            chassis_cmd_send.vx = 0.0f;
            chassis_cmd_send.vy = 0.0f;
            chassis_cmd_send.wz = 0.0f;
        }
    }
    if (chassis_cmd_send.chassis_mode == CHASSIS_MINING)
    {
        if (!(rc_data[TEMP].key[KEY_PRESS].shift) && !(rc_data[TEMP].key[KEY_PRESS].ctrl))
        // W/A/S/D 底盘移动
        {
            chassis_cmd_send.vx = 13000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].a - (float)rc_data[TEMP].key[KEY_PRESS].d); // _水平方向
            chassis_cmd_send.vy = 13000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].w - (float)rc_data[TEMP].key[KEY_PRESS].s); // |竖直方向
            chassis_cmd_send.wz = 1500.0f * ((float)rc_data[TEMP].key[KEY_PRESS].q - (float)rc_data[TEMP].key[KEY_PRESS].e);  // ↺自旋
        }
        else
        {
            chassis_cmd_send.vx = 0.0f;
            chassis_cmd_send.vy = 0.0f;
            chassis_cmd_send.wz = 0.0f;
        }

        upper_cmd_send.joint_data.pitch += -0.01f * (float)rc_data[TEMP].mouse.y;
        upper_cmd_send.joint_data.yaw += 0.01f * (float)rc_data[TEMP].mouse.x;
    }
    if (chassis_cmd_send.chassis_mode == CHASSIS_CHARGE)
    {
        if (!(rc_data[TEMP].key[KEY_PRESS].shift) && !(rc_data[TEMP].key[KEY_PRESS].ctrl))
        // W/A/S/D 底盘移动
        {
            chassis_cmd_send.vx = 4000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].a - (float)rc_data[TEMP].key[KEY_PRESS].d); // _水平方向
            chassis_cmd_send.vy = 4000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].w - (float)rc_data[TEMP].key[KEY_PRESS].s); // |竖直方向
            chassis_cmd_send.wz = 1000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].q - (float)rc_data[TEMP].key[KEY_PRESS].e); // ↺自旋
        }
        else
        {
            chassis_cmd_send.vx = 0.0f;
            chassis_cmd_send.vy = 0.0f;
            chassis_cmd_send.wz = 0.0f;
        }
    }

    // 滑移键鼠控制
    // shift + // W/S // A/D // Q/E
    //           前伸 // 横移 // 抬升
    if ((rc_data[TEMP].key[KEY_PRESS].shift) && !(rc_data[TEMP].key[KEY_PRESS].ctrl))
    {
        upper_cmd_send.joint_data.lift_dist += (1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].q - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].e);
        upper_cmd_send.joint_data.push_dist += (1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].w - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s);
        upper_cmd_send.joint_data.traverse_dist += (0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].a - 0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].d);
    }

    // 机械臂键鼠控制
    //  ctrl + //   W/S  //   A/D  //   Q/E // mouseX // mouseY
    //           小pitch // 小roll // 大roll // 大yaw // 大pitch
    if ((!rc_data[TEMP].key[KEY_PRESS].shift) && (rc_data[TEMP].key[KEY_PRESS].ctrl))
    {
        upper_cmd_send.joint_data.roll += (0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].e - 0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].q);
        upper_cmd_send.joint_data.pitch_differ += (0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w - 0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s);
        upper_cmd_send.joint_data.roll_differ += (0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a - 0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d);
    }

    // 云台键鼠控制
    //  ctrl + shift //   W/S  //  A/D
    //                   pitch //  yaw
    if ((rc_data[TEMP].key[KEY_PRESS].shift) && (rc_data[TEMP].key[KEY_PRESS].ctrl))
    {
        gimbal_cmd_send.yaw_end += (((float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d - (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a));
        gimbal_cmd_send.pitch_end += (((float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s - (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w));
    }

    // 气动元件控制
    // 单击开双击关
    //  ctrl + shift //  z  //   x  //  c  //  zx //  xc  //  zc   //zxc
    //                阀门二//阀门三//阀门四//阀门一//真空泵1//真空泵2//全部
    if ((rc_data[TEMP].key[KEY_PRESS].shift) && (rc_data[TEMP].key[KEY_PRESS].ctrl))
    {
        gimbal_cmd_send.yaw_end += 0.001f * (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d - rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a);
        gimbal_cmd_send.pitch_end += 0.001f * (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s - rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w);
    }

    // Z键打开弹舱
    if (rc_data[TEMP].key[KEY_PRESS].z)
    {
        gimbal_cmd_send.gimbal_mode = Steering_gear_door_open;
        gimbal_cmd_send.damn_mode = DAMN_BACK;
    }
    // ctrl + Z 键关闭弹舱
    if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].z)
    {
        gimbal_cmd_send.damn_mode = DAMN_BACK;
        gimbal_cmd_send.gimbal_mode = Steering_gear_door_close;
    }
    // 单击 R 键打开真空泵
    if (rc_data[TEMP].key[KEY_PRESS].r)
        chassis_cmd_send.pump_mode |= VAVLVE_ARM | VAVLVE_T1 | VAVLVE_T2 | VAVLVE_T3;
    // shift + R 关闭真空泵
    if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].r)
        chassis_cmd_send.pump_mode = VAVLVE_ALL_CLOSE;

    // e舵机模式

    // 5.舵机舱门开
    /*         if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].z)
            {
                gimbal_cmd_send.gimbal_mode = Steering_gear_door_open;
            }
            // 6.舵机舱门关
            if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x)
            {
                gimbal_cmd_send.gimbal_mode = Steering_gear_door_close;
            } */

    if (upper_cmd_send.upper_mode < UPPER_SLIVER_MINING)
    {
        upper_cmd_send.upper_mode = UPPER_SINGLE_MOTOR;

        // ctrl + shift + F 键进入把一位三矿上的银矿吸到机械臂上模式
        // if (rc_data[TEMP].key[KEY_PRESS].f && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].shift)
        // {
        //     upper_cmd_send.upper_mode = UPPER_GET_THREE_SLIVER_MINING;
        // }
        // shift + F 键进入小资源岛一位三矿模式
        if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].f)
        {
            upper_cmd_send.upper_mode = UPPER_THREE_SLIVER_MINING;
        }
        // 单击 F 键进入小资源岛取矿模式
        else if (rc_data[TEMP].key[KEY_PRESS].f)
        {
            upper_cmd_send.upper_mode = UPPER_SLIVER_MINING;
        }
        // 单击 G 键进入大资源岛取矿模式
        if (rc_data[TEMP].key[KEY_PRESS].g)
        {
            upper_cmd_send.upper_mode = UPPER_GLOD_MINING;
            chassis_cmd_send.pump_mode = VAVLVE_ARM;
            upper_cmd_send.joint_data.pitch = 0.0f;
            upper_cmd_send.joint_data.pitch_differ = 0.0f;
        }
        // shift + G 键进入衔接金矿模式
        // if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].g)
        // {
        //     upper_cmd_send.upper_mode = UPPER_GET_GLOD_MINING;
        // }
        // 单击 C 键进入取地面矿模式
        if (rc_data[TEMP].key[KEY_PRESS].c)
        {
            upper_cmd_send.upper_mode = UPPER_GROUND_MINING;
        }
        // 单击 V 键进入自定义控制器兑矿模式
        if (rc_data[TEMP].key[KEY_PRESS].v)
        {
            upper_cmd_send.upper_mode = UPPER_EXCHANGE;
            upper_cmd_send.joint_data.lift_dist = upper_fetch_data.joint_data.lift_dist;
        }
    }

    // 各模式控制任务
    if (upper_cmd_send.upper_mode == UPPER_SLIVER_MINING) // 单银矿石
    {
        chassis_cmd_send.chassis_mode = CHASSIS_MINING;

        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
            upper_cmd_send.stop_flag = 1;
        else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 2) // 再次单击 F 键继续执行
            upper_cmd_send.cfm_flag = 1;

        // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
        if (upper_fetch_data.action_step != 2)
            upper_cmd_send.cfm_flag = 0;

        // 任务执行结束
        if (upper_fetch_data.action_step == 0)
        {
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_THREE_SLIVER_MINING) // 一位三矿
    {
        chassis_cmd_send.chassis_mode = CHASSIS_MINING;
        chassis_cmd_send.pump_mode = VAVLVE_T1 | VAVLVE_T2 | VAVLVE_T3;

        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
            upper_cmd_send.stop_flag = 1;
        else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 3) // 再次单击 F 键继续执行
            upper_cmd_send.cfm_flag = 3;
        else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 5) // 再次单击 F 键继续执行
        {
            upper_cmd_send.cfm_flag = 1;
            gimbal_cmd_send.gimbal_mode = GIMBAL_NORMAL;
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            gimbal_cmd_send.yaw = 3;
            gimbal_cmd_send.pitch = 90;
            gimbal_cmd_send.yaw_end = 0;
            gimbal_cmd_send.pitch_end = 0;
        }
            

        if (upper_fetch_data.action_step != 3 && upper_fetch_data.action_step != 5)
            upper_cmd_send.cfm_flag = 0;
        // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
        // 任务执行结束
        if (upper_fetch_data.action_step == 0)
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_GET_THREE_SLIVER_MINING) // 一位三矿交接
    {
        chassis_cmd_send.chassis_mode = CHASSIS_MINING;

        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x && !rc_data[TEMP].key[KEY_PRESS].shift) // 退出模式
            upper_cmd_send.stop_flag = 1;
        if (getsliver_flag % 3 == 0)
        {
            if (upper_fetch_data.action_step == 2)
                chassis_cmd_send.pump_mode = VAVLVE_T1 | VAVLVE_T2 | VAVLVE_T3;
            if (upper_fetch_data.action_step == 3)
                chassis_cmd_send.pump_mode = VAVLVE_T2 | VAVLVE_T3;
        }
        else if (getsliver_flag % 3 == 1)
        {
            if (upper_fetch_data.action_step == 2)
                chassis_cmd_send.pump_mode = VAVLVE_T2 | VAVLVE_T3;
            if (upper_fetch_data.action_step == 3)
                chassis_cmd_send.pump_mode = VAVLVE_T3;
        }
        else if (getsliver_flag % 3 == 2)
        {
            if (upper_fetch_data.action_step == 2)
                chassis_cmd_send.pump_mode = VAVLVE_T3;
            if (upper_fetch_data.action_step == 3)
                chassis_cmd_send.pump_mode = VAVLVE_ALL_CLOSE;
        }
        // 任务执行结束
        if (upper_fetch_data.action_step == 0)
        {
            getsliver_flag++;
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.getsliver_flag = getsliver_flag;
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_GLOD_MINING) // 取金矿石
    {
        chassis_cmd_send.chassis_mode = CHASSIS_MINING;

        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
        {
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_GET_GLOD_MINING) // 金矿石衔接模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_MINING;

        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
            upper_cmd_send.stop_flag = 1;

        // 任务执行结束
        if (upper_fetch_data.action_step == 0)
        {
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_GROUND_MINING) // 地面矿石
    {
        chassis_cmd_send.chassis_mode = CHASSIS_MINING;

        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
            upper_cmd_send.stop_flag = 1;
        else if (rc_data[TEMP].key[KEY_PRESS].c && upper_fetch_data.action_step == 2) // 再次单击 C 键继续执行
            upper_cmd_send.cfm_flag = 1;

        if (upper_fetch_data.action_step == 1)
        {
            chassis_cmd_send.pump_mode = VAVLVE_ARM;
        }
        // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
        if (upper_fetch_data.action_step != 2)
            upper_cmd_send.cfm_flag = 0;

        // 任务执行结束
        if (upper_fetch_data.action_step == 0)
        {
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_EXCHANGE) // 控制器兑换
    {
        chassis_cmd_send.chassis_mode = CHASSIS_MINING;

        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
        {
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
        }
    }

    gimbal_cmd_send.yaw_end = int16_constrain(gimbal_cmd_send.yaw_end, 0 - gimbal_cmd_send.yaw, 270 - gimbal_cmd_send.yaw);
    gimbal_cmd_send.pitch_end = int16_constrain(gimbal_cmd_send.pitch_end, 90 - gimbal_cmd_send.pitch, 180 - gimbal_cmd_send.pitch);
}
/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @return 1-正常运行
 *         0-急停模式
 *
 */
static uint8_t EmergencyHandler()
{
    // 遥控器右侧开关为[下]进入急停模式.
    if (switch_is_down(rc_data[TEMP].rc.switch_right) || !(RemoteControlIsOnline())) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        chassis_cmd_send.pump_mode = VAVLVE_ALL_CLOSE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        upper_cmd_send.upper_mode = UPPER_ZERO_FORCE;
        upper_cmd_send.stop_flag = 0;
        upper_cmd_send.cfm_flag = 0;
        LOGERROR("[CMD] emergency stop!");
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        gimbal_cmd_send.damn_mode = DAMN_KEEP;
        gimbal_cmd_send.yaw = 90;
        gimbal_cmd_send.pitch = 90;
        gimbal_cmd_send.yaw_end = 0;
        gimbal_cmd_send.pitch_end = 0;
        CmdRecvUpdate();

        return 0;
    }
    // 恢复正常运行
    else
    {
        robot_state = ROBOT_READY;
        // LOGINFO("[CMD] reinstate, robot ready");

        return 1;
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
    gimbal_fetch_data = *(Gimbal_Upload_Data_s *)CANCommGet(cam_can_gimbal);
    memcpy(&upper_cmd_send.ctrlr_data, &chassis_fetch_data.ctrlr_data, sizeof(Self_Cntlr_s));
#endif // GIMBAL_BOARD
    // SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(upper_feed_sub, &upper_fetch_data);

    if (EmergencyHandler()) // 处理模块离线和遥控器急停等紧急情况
    {
        if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_mid(rc_data[TEMP].rc.switch_right))
            MouseKeyControlSet(); // 键鼠控制
        else
            RemoteControlSet(); // 遥控器控制

        // 电管云台断电
        if (!chassis_fetch_data.GameRobotState.power_management_gimbal_output)
        {
            upper_cmd_send.upper_mode = UPPER_ZERO_FORCE;
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.cfm_flag = 0;
            CmdRecvUpdate();
        }
        // 电管底盘断电
        if (!chassis_fetch_data.GameRobotState.power_management_chassis_output)
        {
            chassis_cmd_send.pump_mode = VAVLVE_ALL_CLOSE;
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        }
    }

    UpperJointConstrain(&upper_cmd_send.joint_data);

    if (upper_last_mode != upper_cmd_send.upper_mode)
    {
        CmdRecvUpdate();
    }
    upper_last_mode = upper_cmd_send.upper_mode;

    chassis_cmd_send.gimbal_mode = gimbal_cmd_send.gimbal_mode;
    chassis_cmd_send.upper_mode = upper_cmd_send.upper_mode;
    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
    CANCommSend(cam_can_gimbal, (void *)&gimbal_cmd_send);
#endif // GIMBAL_BOARD
    // PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(upper_cmd_pub, (void *)&upper_cmd_send);
}
