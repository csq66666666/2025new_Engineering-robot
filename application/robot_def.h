/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "self_controller.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
// #define ONE_BOARD // 单板控制整车
#define CHASSIS_BOARD // 底盘板
// #define GIMBAL_BOARD // 云台板

#define VISION_USE_VCP // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 机械臂参数
#define CALI_STEP_TIME 100                                              // 校准时间
#define ACTION_STEP_TIME 50                                             // 动作完成判断时间
#define YAW_MAX 50.0f                                                   // YAW轴最大角度
#define YAW_MIN -50.0f                                                  // YAW轴最小角度
#define ROLL_MAX 110.0f                                                 // roll轴最大角度
#define ROLL_MIN -110.0f                                                // roll轴最小角度
#define PITCH_MAX 18.0f                                                 // pitch轴最大角度
#define PITCH_MIN -88.0f                                                // pitch轴最小角度
#define PITCH_DIFFER_MAX 90.0f                                          // pitch_differ轴最大角度
#define PITCH_DIFFER_MIN -90.0f                                         // pitch_differ轴最小角度
#define GEAR_RATION_YAW (14.0f / 54.0f)                                 // yaw轴齿轮比
#define GEAR_RATION_ROLL (10.0f / 38.0f)                                // roll轴齿轮比
#define GEAR_RATION_PITCH (16.0f / 32.0f)                               // pitch轴齿轮比
#define GEAR_RATION_DIFFER (30.0f / 50.0f)                              // 差速器太阳齿轮:行星齿轮
#define REDUCTION_RATIO_YAW 36.0f                                       // yaw轴电机减速比
#define REDUCTION_RATIO_ROLL 36.0f                                      // roll轴电机减速比
#define REDUCTION_RATIO_PITCH 19.0f                                     // pitch轴电机减速比
#define REDUCTION_RATIO_DIFFER 36.0f                                    // 差速器电机减速比
#define ROTOR_2_SHAFT_YAW (GEAR_RATION_YAW / REDUCTION_RATIO_YAW)       // 电机转子角度转换到yaw轴角度
#define ROTOR_2_SHAFT_ROLL (GEAR_RATION_ROLL / REDUCTION_RATIO_ROLL)    // 电机转子角度转换到roll轴角度
#define ROTOR_2_SHAFT_PITCH (GEAR_RATION_PITCH / REDUCTION_RATIO_PITCH) // 电机转子角度转换到pitch轴角度
#define ROTOR_2_SHAFT_ROLL_DIFFER (1.0f / REDUCTION_RATIO_DIFFER)       // 电机转子角度转换到差速器roll轴角度
#define ROTOR_2_SHAFT_PITCH_DIFFER (1.0f / REDUCTION_RATIO_DIFFER)      // 电机转子角度转换到差速器pitch轴角度
#define SHAFT_2_ROTOR_YAW (REDUCTION_RATIO_YAW / GEAR_RATION_YAW)       // yaw轴角度转换到电机转子角度
#define SHAFT_2_ROTOR_ROLL (REDUCTION_RATIO_ROLL / GEAR_RATION_ROLL)    // roll轴角度转换到电机转子角度
#define SHAFT_2_ROTOR_PITCH (REDUCTION_RATIO_PITCH / GEAR_RATION_PITCH) // pitch轴角度转换到电机转子角度
#define SHAFT_2_ROTOR_ROLL_DIFFER REDUCTION_RATIO_DIFFER                // 差速器roll轴角度转换到电机转子角度
#define SHAFT_2_ROTOR_PITCH_DIFFER REDUCTION_RATIO_DIFFER               // 差速器pitch轴角度转换到电机转子角度
// 滑移参数
#define LIFT_GEAR_R 23.3f                                             // 抬升齿轮半径
#define PUSH_GEAR_R 13.945f                                           // 前伸齿轮半径
#define TRAVERSE_GEAR_R 10.13f                                        // 横移齿轮半径
#define PUSH_MAX_DIST 250.0f                                          // 前伸最大距离 250
#define LIFT_MAX_DIST 529.0f                                          // 抬升最大距离
#define TRAVERSE_MAX_DIST 360.0f                                      // 横移最大距离
#define LIFT_DIST_2_ANGLE (360.0f * 19.0f / (2.0f * PI * 23.3f))      // 抬升距离转电机角度
#define PUSH_DIST_2_ANGLE (360.0f * 19.0f / (2.0f * PI * 13.945f))    // 前伸距离转电机角度
#define TRAVERSE_DIST_2_ANGLE (360.0f * 36.0f / (2.0f * PI * 10.13f)) // 横移距离转电机角度
#define SAFE_DIST 10.0f                                               // 安全距离
#define LIFT_MAX_SAFE_DIST (LIFT_MAX_DIST - SAFE_DIST)                // 抬升最大安全距离
#define PUSH_MAX_SAFE_DIST (PUSH_MAX_DIST - SAFE_DIST)                // 前伸最大安全距离
#define TRAVERSE_MAX_SAFE_DIST (TRAVERSE_MAX_DIST - SAFE_DIST)        // 横移最大安全距离
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 2711  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 0           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE 0           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 49.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 350                                                    // 纵向轴距(前进后退方向)
#define WHEEL_TRACK 300                                                   // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0                                          // 云台旋转中心距底盘几何中心的距离,左右方向,向右为正方向，云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0                                          // 云台旋转中心距底盘几何中心的距离,前后方向,向左为正方向，云台位于正中心时默认设为0
#define RADIUS_WHEEL 77                                                   // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f                                       // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define N2V ((2 * PI * RADIUS_WHEEL) / (REDUCTION_RATIO_WHEEL * 60000.0)) // 电机转子转速转轮子转速
#define V2N ((REDUCTION_RATIO_WHEEL * 60000.0) / (2 * PI * RADIUS_WHEEL)) // 轮子转速转电机转子转速
#define SMOOTH_COEF_CHASSIS 0.008f                                        // 底盘电机使用的低通滤波器系数
#define MIN_ACCEL (6 * 3.0f * 0.005f * V2N)
#define MAX_ACCEL (6 * 3.0f * 0.005f * V2N)

#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// 电磁阀状态
#define VAVLVE_ALL_CLOSE ((uint8_t)0b0000) // 关闭全部阀门
#define VAVLVE_ARM ((uint8_t)0b1000)       // 选中机械臂阀门
#define VAVLVE_T1 ((uint8_t)0b0100)        // 选中横移第一路阀门
#define VAVLVE_T2 ((uint8_t)0b0010)        // 选中横移第二路阀门
#define VAVLVE_T3 ((uint8_t)0b0001)        // 选中横移第三路阀门

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0, // 电流零输入
    // referee_task.h 中有用到这三个枚举值，为了避免出bug，目前先保留，后期调试裁判系统时再做更改
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_NORMAL,            // 正常行进模式
    CHASSIS_MINING,            // 取矿行进模式
    CHASSIS_CHARGE,            // 补弹行进模式
    CHASSIS_NO_MOVE,           // 锁定底盘
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_FREE_MODE,
    GIMBAL_DAMN_MODE,
    GIMBAL_NORMAL,
    GIMBAL_MINING,
    Steering_gear_door_open,
    Steering_gear_door_close,
} gimbal_mode_e;

// 云台角度控制
typedef enum
{
    DAMN_KEEP = 0, // 保持当前角度
    DAMN_BACK,
} DANM_mode_e;

// 上层机构模式设置
typedef enum
{
    UPPER_ZERO_FORCE = 0,          // 电流零输入
    UPPER_NO_MOVE,                 // 锁定上层机构
    UPPER_CALI,                    // 校准模式
    UPPER_SINGLE_MOTOR,            // 单电机控制模式
    UPPER_SLIVER_MINING,           // 开采银矿模式
    UPPER_THREE_SLIVER_MINING,     // 一键开采三个银矿模式
    UPPER_GET_THREE_SLIVER_MINING, // 一键衔接三个银矿模式
    UPPER_GLOD_MINING,             // 开采金矿模式
    UPPER_GET_GLOD_MINING,         // 衔接金矿模式
    UPPER_GROUND_MINING,           // 开采地面矿模式
    UPPER_EXCHANGE,                // 兑换模式
} upper_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

// 上层机构关节数据
typedef struct
{
    float yaw;
    float roll;
    float pitch;
    float pitch_differ;
    float roll_differ;

    float lift_dist;
    float push_dist;
    float traverse_dist;
} Upper_Joint_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot/cmd订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float yaw_end;
    float pitch_end;

    gimbal_mode_e gimbal_mode;
    DANM_mode_e damn_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

// cmd发布的上层机构控制数据,由upper订阅
typedef struct
{
    Upper_Joint_Data_s joint_data; // 关节数据
    Self_Cntlr_s ctrlr_data;       // 自定义控制器数据

    uint8_t cfm_flag : 4;
    uint8_t stop_flag : 1;
    uint16_t getsliver_flag;
    upper_mode_e upper_mode;
} Upper_Ctrl_Cmd_s;

// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx; // 前进方向速度
    float vy; // 横移方向速度
    float wz; // 旋转速度

    chassis_mode_e chassis_mode;
    uint8_t pump_mode;

    // UI部分
    upper_mode_e upper_mode;
    gimbal_mode_e gimbal_mode;
} Chassis_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis/upper发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif

    Self_Cntlr_s ctrlr_data;               // 自定义控制器数据
    ext_game_robot_state_t GameRobotState; // 裁判系统数据
} Chassis_Upload_Data_s;

typedef struct
{
    // attitude_t gimbal_imu_data;
    // uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct
{
    // code to go here
    // ...
} Shoot_Upload_Data_s;

typedef struct
{
    Upper_Joint_Data_s joint_data; // 上层机构关节数据
    uint8_t action_step;
} Upper_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H