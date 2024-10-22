/**
 * @file chassis.c
 * @author liyong@stu.cdu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version final
 * @date 2024-05-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "referee_task.h"
#include "self_controller.h"
#include "elec_switch.h"
#include "vofa.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_WHEEL_TRACK (WHEEL_TRACK / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                                                                  // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                                                                 // 用于订阅底盘的控制命令
#endif                                                                                            // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;                                                       // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data;                                               // 底盘回传的反馈数据
static referee_info_t *referee_data;                                                              // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data;                                                        // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb, *motor_lift_l, *motor_lift_r; // left right forward back
static Self_Cntlr_s *self_cntlr_data;
static ElecSwitchInstance *valve_1, *valve_2, *valve_3, *valve_4, *pump;    // 4个继电器加2个霍尔开关
static float vofa_data[8];

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float sin_theta, cos_theta;   // 设置底盘行进方向
static float chassis_vx, chassis_vy; // 将云台系的速度投影到底盘
static float chassis_vx_m, chassis_vy_m, chassis_wz_m;
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 1.98, // 4.5
                .Ki = 0,    // 0
                .Kd = 0,    // 0
                .IntegralLimit = 8000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };

    // 当底盘处于前进速度的时候，电机1 和电机4 是顺时针旋转为负值，电机2 和电机3 是逆时针旋转为正值，所以将电机1和4的输出取反值
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    referee_data = UITaskInit(&huart1, &ui_data); // 裁判系统初始化,会同时初始化UI（注意自定义控制器使用了学生串口huart6，我们的裁判系统接口为huart1）
    self_cntlr_data = SelfCntlrInit(&huart6); // 自定义控制器初始化
    
    // 电磁阀控制端口初始化
    ElecSwitch_Init_Config_s valve_init_cofig = {
        .GPIOx = VALVE1_GPIO_Port,
        .GPIO_Pin = VALVE1_Pin,
        .GPIO_Mode = GPIO_MODE_OUTPUT_PP,
        .trigger_level = HIGH,
    };
    valve_1 = ElecSwitchInit(&valve_init_cofig);
    valve_init_cofig.GPIOx = VALVE2_GPIO_Port;
    valve_init_cofig.GPIO_Pin = VALVE2_Pin;
    valve_2 = ElecSwitchInit(&valve_init_cofig);
    valve_init_cofig.GPIOx = VALVE3_GPIO_Port;
    valve_init_cofig.GPIO_Pin = VALVE3_Pin;
    valve_3 = ElecSwitchInit(&valve_init_cofig);
    valve_init_cofig.GPIOx = VALVE4_GPIO_Port;
    valve_init_cofig.GPIO_Pin = VALVE4_Pin;
    valve_4 = ElecSwitchInit(&valve_init_cofig);

    valve_init_cofig.GPIOx = HALL_GPIO_Port;
    valve_init_cofig.GPIO_Pin = HALL1_Pin;
    pump = ElecSwitchInit(&valve_init_cofig);

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_WHEEL_TRACK + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_WHEEL_TRACK - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_WHEEL_TRACK + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_WHEEL_TRACK - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 正运动学解算
 */
static void MecanumFKine()
{
    chassis_vx_m = (motor_rf->measure.speed_aps + motor_lf->measure.speed_aps - motor_lb->measure.speed_aps - motor_rb->measure.speed_aps) / 4.0f;
    chassis_vy_m = (-motor_rf->measure.speed_aps + motor_lf->measure.speed_aps + motor_lb->measure.speed_aps - motor_rb->measure.speed_aps) / 4.0f;
    chassis_wz_m = (-motor_rf->measure.speed_aps - motor_lf->measure.speed_aps - motor_lb->measure.speed_aps - motor_rb->measure.speed_aps) / (LF_CENTER + RF_CENTER + LB_CENTER + RB_CENTER);
}

/**
 * @brief 逆运动学解算，计算每个轮毂电机的输出
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumIKine()
{
    vt_rf = -chassis_vx + chassis_vy + chassis_cmd_recv.wz * RF_CENTER; // 1
    vt_lf = chassis_vx + chassis_vy - chassis_cmd_recv.wz * LF_CENTER;  // 2
    vt_lb = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * LB_CENTER; // 3
    vt_rb = chassis_vx + chassis_vy + chassis_cmd_recv.wz * RB_CENTER;  // 4
}

/**
 * @brief 底盘控制量输出
 *
 */
static void ChassisOutput()
{
    // 完成加速度限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 底盘回传的反馈数据：自定义控制器数据、机器人状态数据
 *
 */
static void FeedbackUpdate()
{
    chassis_feedback_data.ctrlr_data.yaw = 4.5f * self_cntlr_data->yaw;
    chassis_feedback_data.ctrlr_data.pitch = self_cntlr_data->pitch;
    chassis_feedback_data.ctrlr_data.roll = -self_cntlr_data->roll;
    chassis_feedback_data.ctrlr_data.push_dist = self_cntlr_data->push_dist;
    chassis_feedback_data.ctrlr_data.traverse_dist = self_cntlr_data->traverse_dist;
    chassis_feedback_data.GameRobotState = referee_data->GameRobotState;
}

/* static void AccelLimit(float *velocity, float *measure, float max_accel)
{
    static uint32_t time_count;
    time_count++;
    if ((*velocity - *measure) > max_accel)
    {
        *velocity = *measure + max_accel * time_count;
    }
    else if ((*velocity - *measure) < -max_accel)
    {
        *velocity = *measure - max_accel * time_count;
    }
} */

/**
 * @brief 底盘加速度限幅
 *
 */
static void AccelLimit()
{
    // @todo: 把此功能封装成一个函数能够被多次调用
    static uint32_t time_count_vx, time_count_vy, time_count_wz;
    static float chassis_vx_set, chassis_vy_set, chassis_wz_set;
    static uint8_t last_mode_vx, last_mode_vy, last_mode_wz;
    static float delta_speed_max;
    // delta_speed_max = (MAX_ACCEL - MIN_ACCEL) * K + MIN_ACCEL;

    if ((chassis_vx - chassis_vx_m) > MAX_ACCEL)
    {
        if (last_mode_vx != 1)
        {
            time_count_vx = 0;
            chassis_vx_set = chassis_vx_m;
        }
        time_count_vx++;
        chassis_vx = chassis_vx_set + MAX_ACCEL * time_count_vx;
        last_mode_vx = 1;
    }
    else if ((chassis_vx - chassis_vx_m) < -MAX_ACCEL)
    {
        if (last_mode_vx != 2)
        {
            time_count_vx = 0;
            chassis_vx_set = chassis_vx_m;
        }
        time_count_vx++;
        chassis_vx = chassis_vx_set - MAX_ACCEL * time_count_vx;
        last_mode_vx = 2;
    }
    else
    {
        last_mode_vx = 3;
    }

    if ((chassis_vy - chassis_vy_m) > MAX_ACCEL)
    {
        if (last_mode_vy != 1)
        {
            time_count_vy = 0;
            chassis_vy_set = chassis_vy_m;
        }
        time_count_vy++;
        chassis_vy = chassis_vy_set + MAX_ACCEL * time_count_vy;
        last_mode_vy = 1;
    }
    else if ((chassis_vy - chassis_vy_m) < -MAX_ACCEL)
    {
        if (last_mode_vy != 2)
        {
            time_count_vy = 0;
            chassis_vy_set = chassis_vy_m;
        }
        time_count_vy++;
        chassis_vy = chassis_vy_set - MAX_ACCEL * time_count_vy;
        last_mode_vy = 2;
    }
    else
    {
        last_mode_vy = 3;
    }

    if ((chassis_cmd_recv.wz - chassis_wz_m) > MAX_ACCEL)
    {
        if (last_mode_wz != 1)
        {
            time_count_wz = 0;
            chassis_wz_set = chassis_wz_m;
        }
        time_count_wz++;
        chassis_cmd_recv.wz = chassis_wz_set + MAX_ACCEL * time_count_wz;
        last_mode_wz = 1;
    }
    else if ((chassis_cmd_recv.wz - chassis_wz_m) < -MAX_ACCEL)
    {
        if (last_mode_wz != 2)
        {
            time_count_wz = 0;
            chassis_wz_set = chassis_wz_m;
        }
        time_count_wz++;
        chassis_cmd_recv.wz = chassis_wz_set - MAX_ACCEL * time_count_wz;
        last_mode_wz = 2;
    }
    else
    {
        last_mode_wz = 3;
    }
}

/**
 * @brief 用于将收到的ui数据更新
 *
 */
static void ui_feedup()
{
    ui_data.pump_mode = chassis_cmd_recv.pump_mode;
    ui_data.chassis_mode = chassis_cmd_recv.chassis_mode;
    ui_data.upper_mode = chassis_cmd_recv.upper_mode;
    ui_data.gimbal_mode = chassis_cmd_recv.gimbal_mode;
}

/**
 * @brief 用于ui在裁判系统上电后初始化
 * @note  可能有更好的实现方法
 */
static void UI_INIT_SECOND()
{
    static int hasEnteredFunction1 = 0; // 0表示还没有进入过函数1

    if (motor_lf->measure.temperature != 0 && hasEnteredFunction1 == 0)
    {
        MyUIInit();              // 这是你要调用的函数1
        hasEnteredFunction1 = 1; // 更新标志，表示已经进入过函数1
    }
}

/**
 * @brief 控制真空泵和电磁阀
 *
 */
static void ElecSwitchControl()
{
    (chassis_cmd_recv.pump_mode & VAVLVE_ARM) ? ElecSwitchSet(valve_1) : ElecSwitchReset(valve_1);
    (chassis_cmd_recv.pump_mode & VAVLVE_T1) ? ElecSwitchSet(valve_2) : ElecSwitchReset(valve_2);
    (chassis_cmd_recv.pump_mode & VAVLVE_T2) ? ElecSwitchSet(valve_3) : ElecSwitchReset(valve_3);
    (chassis_cmd_recv.pump_mode & VAVLVE_T3) ? ElecSwitchSet(valve_4) : ElecSwitchReset(valve_4);
    chassis_cmd_recv.pump_mode ? ElecSwitchSet(pump) : ElecSwitchReset(pump);
}

/**
 * @brief 根据底盘模式选择控制方式
 *
 */
static void ChassisModeControl()
{
    DJIMotorEnable(motor_lf);
    DJIMotorEnable(motor_lb);
    DJIMotorEnable(motor_rf);
    DJIMotorEnable(motor_rb);

    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_ZERO_FORCE: // 正常行进
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_rb);
        break;
    case CHASSIS_NORMAL: // 正常行进
        cos_theta = -1;
        sin_theta = 0;
        break;
    case CHASSIS_MINING: // 取矿行进
        cos_theta = 0;
        sin_theta = -1;
        break;
    case CHASSIS_CHARGE: // 补弹行进
        cos_theta = 0;
        sin_theta = 1;
        break;
    case CHASSIS_NO_MOVE: // 锁定底盘
        chassis_cmd_recv.vx = 0;
        chassis_cmd_recv.vy = 0;
        chassis_cmd_recv.wz = 0;
        break;
    default:
        chassis_cmd_recv.vx = 0;
        chassis_cmd_recv.vy = 0;
        chassis_cmd_recv.wz = 0;
        break;
    }

    // 底盘向右为左右正方向，向前为逆时针旋转为角度正方向;
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    // 电磁阀控制
    ElecSwitchControl();

    // 根据控制模式设定底盘速度
    ChassisModeControl();

    // 正运动学解算
    MecanumFKine();

    // 加速度限幅
    AccelLimit();

    // 根据控制模式进行逆运动学解算,计算底盘输出
    MecanumIKine();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    ChassisOutput();

    // 底盘回传的反馈数据
    FeedbackUpdate();

    // 用于将收到的ui数据更新
    ui_feedup();

    // UI_INIT_SECOND();

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}