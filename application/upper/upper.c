#include "upper.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "general_def.h"
#include "vofa.h"

// 上层机构所有电机定义
static DJIMotorInstance *upper_yaw_motor, *upper_roll_motor, *upper_pitch_motor, *upper_differ_motor_l, *upper_differ_motor_r;
static DJIMotorInstance *upper_lift_motor_l, *upper_lift_motor_r, *upper_push_motor_l, *upper_push_motor_r, *upper_traverse_motor;

static float upper_yaw_op, upper_roll_op, upper_pitch_op, upper_differ_l_op, upper_differ_r_op;     // 机械臂电机输出数据
static float upper_lift_l_op, upper_lift_r_op, upper_push_l_op, upper_push_r_op, upper_traverse_op; // 滑移电机输出数据
static upper_mode_e upper_last_mode;                                                                // 上一次的模式
static Upper_Joint_Data_s upper_solve;                                                              // 上层机构所有关节数据
static Upper_Kine_s Upper_Kine;                                                                     // 运动学数据
static uint8_t action_finish_flag;
static uint8_t action_step = 0;
static uint8_t action_flag = 0;

static Publisher_t *upper_pub;                  // 机械臂应用消息发布者(机械臂反馈给cmd)
static Subscriber_t *upper_sub;                 // cmd控制消息订阅者
static Upper_Upload_Data_s upper_feedback_data; // 回传给cmd的机械臂状态信息
static Upper_Ctrl_Cmd_s upper_cmd_recv;         // 来自cmd的控制信息

attitude_t *Upper_IMU_data;

/**
 * @brief 通过电机速度来判断动作是否完成
 * @param step 完成一次 加一
 * @param cmd_time 当前堵转时间
 * @param dirt 执行方向 1: +； 0: -;
 * @param step_time 堵转完成时间
 * @note  电机堵转速度不用设置很小，因为电机运动能轻易达到几百上千rpm
 */
#define ActionFinishJudge(step, cmd_time, dirt, step_time)          \
    {                                                               \
        if (fabsf(upper_yaw_motor->measure.speed_aps) < 100 &&      \
            fabsf(upper_roll_motor->measure.speed_aps) < 100 &&     \
            fabsf(upper_pitch_motor->measure.speed_aps) < 100 &&    \
            fabsf(upper_differ_motor_l->measure.speed_aps) < 100 && \
            fabsf(upper_differ_motor_r->measure.speed_aps) < 100 && \
            fabsf(upper_lift_motor_l->measure.speed_aps) < 100 &&   \
            fabsf(upper_lift_motor_r->measure.speed_aps) < 100 &&   \
            fabsf(upper_push_motor_l->measure.speed_aps) < 100 &&   \
            fabsf(upper_push_motor_r->measure.speed_aps) < 100 &&   \
            fabsf(upper_traverse_motor->measure.speed_aps) < 100)   \
        {                                                           \
            (cmd_time)++;                                           \
            if ((cmd_time) > (step_time))                           \
            {                                                       \
                (cmd_time) = 0;                                     \
                if (dirt)                                           \
                    (step)++;                                       \
                else                                                \
                    (step)--;                                       \
            }                                                       \
        }                                                           \
    }

void UpperInit()
{
    Upper_IMU_data = INS_Init(); // 云台IMU初始化
    // yaw轴电机
    Motor_Init_Config_s upper_motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2000,
                .MaxOut = 15000,
            },
            .speed_PID = {
                .Kp = 1,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };
    upper_yaw_motor = DJIMotorInit(&upper_motor_config);
    upper_yaw_motor->measure.init_flag = 1;

    // 大roll轴电机
    upper_motor_config.can_init_config.can_handle = &hcan1;
    upper_motor_config.can_init_config.tx_id = 2;
    upper_motor_config.controller_param_init_config.angle_PID.Kp = 10;
    upper_motor_config.controller_param_init_config.angle_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.angle_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.angle_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.angle_PID.MaxOut = 15000;
    upper_motor_config.controller_param_init_config.speed_PID.Kp = 2;
    upper_motor_config.controller_param_init_config.speed_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.speed_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.speed_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.speed_PID.MaxOut = 15000;
    upper_roll_motor = DJIMotorInit(&upper_motor_config);
    upper_roll_motor->measure.init_flag = 1;

    // 大pitch轴电机
    upper_motor_config.can_init_config.can_handle = &hcan1;
    upper_motor_config.can_init_config.tx_id = 3;
    upper_motor_config.controller_param_init_config.angle_PID.Kp = 15;
    upper_motor_config.controller_param_init_config.angle_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.angle_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.angle_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.angle_PID.MaxOut = 10000;
    upper_motor_config.controller_param_init_config.speed_PID.Kp = 4;
    upper_motor_config.controller_param_init_config.speed_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.speed_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.speed_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.speed_PID.MaxOut = 15000;
    upper_motor_config.motor_type = M3508;
    upper_pitch_motor = DJIMotorInit(&upper_motor_config);
    upper_pitch_motor->measure.init_flag = 1;

    // 差速器左侧电机
    upper_motor_config.can_init_config.can_handle = &hcan1;
    upper_motor_config.can_init_config.tx_id = 4;
    upper_motor_config.controller_param_init_config.angle_PID.Kp = 10;
    upper_motor_config.controller_param_init_config.angle_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.angle_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.angle_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.angle_PID.MaxOut = 10000;
    upper_motor_config.controller_param_init_config.speed_PID.Kp = 3;
    upper_motor_config.controller_param_init_config.speed_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.speed_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.speed_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.speed_PID.MaxOut = 15000;
    upper_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    upper_motor_config.motor_type = M2006;
    upper_differ_motor_l = DJIMotorInit(&upper_motor_config);
    upper_differ_motor_l->measure.init_flag = 1;

    // 差速器右侧电机
    upper_motor_config.can_init_config.can_handle = &hcan1;
    upper_motor_config.can_init_config.tx_id = 5;
    upper_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    upper_differ_motor_r = DJIMotorInit(&upper_motor_config);
    upper_differ_motor_r->measure.init_flag = 1;

    // 抬升左侧电机
    upper_motor_config.can_init_config.can_handle = &hcan2;
    upper_motor_config.can_init_config.tx_id = 1;
    upper_motor_config.controller_param_init_config.angle_PID.Kp = 14;
    upper_motor_config.controller_param_init_config.angle_PID.Ki = 3.05;
    upper_motor_config.controller_param_init_config.angle_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.angle_PID.IntegralLimit = 8000;
    upper_motor_config.controller_param_init_config.angle_PID.MaxOut = 12000;
    upper_motor_config.controller_param_init_config.speed_PID.Kp = 1.2;
    upper_motor_config.controller_param_init_config.speed_PID.Ki = 6;
    upper_motor_config.controller_param_init_config.speed_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.speed_PID.IntegralLimit = 5000;
    upper_motor_config.controller_param_init_config.speed_PID.MaxOut = 15000;
    upper_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    upper_motor_config.motor_type = M3508;
    upper_lift_motor_l = DJIMotorInit(&upper_motor_config);
    upper_lift_motor_l->measure.init_flag = 1;

    // 抬升右侧电机
    upper_motor_config.can_init_config.can_handle = &hcan2;
    upper_motor_config.can_init_config.tx_id = 2;
    upper_motor_config.controller_param_init_config.angle_PID.Kp = 14;
    upper_motor_config.controller_param_init_config.angle_PID.Ki = 3;
    upper_motor_config.controller_param_init_config.angle_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.angle_PID.IntegralLimit = 8000;
    upper_motor_config.controller_param_init_config.angle_PID.MaxOut = 12000;
    upper_motor_config.controller_param_init_config.speed_PID.Kp = 1.215;
    upper_motor_config.controller_param_init_config.speed_PID.Ki = 6;
    upper_motor_config.controller_param_init_config.speed_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.speed_PID.IntegralLimit = 5000;
    upper_motor_config.controller_param_init_config.speed_PID.MaxOut = 15000;
    upper_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    upper_lift_motor_r = DJIMotorInit(&upper_motor_config);
    upper_lift_motor_r->measure.init_flag = 1;

    // 前伸左侧电机
    upper_motor_config.can_init_config.can_handle = &hcan2;
    upper_motor_config.can_init_config.tx_id = 3;
    upper_motor_config.controller_param_init_config.angle_PID.Kp = 10;
    upper_motor_config.controller_param_init_config.angle_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.angle_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.angle_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.angle_PID.MaxOut = 12000;
    upper_motor_config.controller_param_init_config.speed_PID.Kp = 1.5;
    upper_motor_config.controller_param_init_config.speed_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.speed_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.speed_PID.IntegralLimit = 5000;
    upper_motor_config.controller_param_init_config.speed_PID.MaxOut = 15000;
    upper_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    upper_motor_config.motor_type = M3508;
    upper_push_motor_l = DJIMotorInit(&upper_motor_config);
    upper_push_motor_l->measure.init_flag = 1;

    // 前伸右侧电机
    upper_motor_config.can_init_config.can_handle = &hcan2;
    upper_motor_config.can_init_config.tx_id = 4;
    upper_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    upper_push_motor_r = DJIMotorInit(&upper_motor_config);
    upper_push_motor_r->measure.init_flag = 1;

    // 横移电机
    upper_motor_config.can_init_config.can_handle = &hcan1;
    upper_motor_config.can_init_config.tx_id = 6;
    upper_motor_config.controller_param_init_config.angle_PID.Kp = 10;
    upper_motor_config.controller_param_init_config.angle_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.angle_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.angle_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.angle_PID.MaxOut = 20000;
    upper_motor_config.controller_param_init_config.speed_PID.Kp = 2.1;
    upper_motor_config.controller_param_init_config.speed_PID.Ki = 0.21;
    upper_motor_config.controller_param_init_config.speed_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.speed_PID.IntegralLimit = 3000;
    upper_motor_config.controller_param_init_config.speed_PID.MaxOut = 15000;
    upper_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    upper_motor_config.motor_type = M2006;
    upper_traverse_motor = DJIMotorInit(&upper_motor_config);
    upper_traverse_motor->measure.init_flag = 1;

    upper_pub = PubRegister("upper_feed", sizeof(Upper_Upload_Data_s));
    upper_sub = SubRegister("upper_cmd", sizeof(Upper_Ctrl_Cmd_s));
}

/**
 * @brief 将各关节当前状态反馈给cmd
 *
 */
static void UpperFeedUpdata()
{
    upper_feedback_data.joint_data.yaw = (upper_yaw_motor->measure.total_angle - upper_yaw_motor->measure.init_angle) * ROTOR_2_SHAFT_YAW;
    upper_feedback_data.joint_data.roll = (upper_roll_motor->measure.total_angle - upper_roll_motor->measure.init_angle) * ROTOR_2_SHAFT_ROLL;
    upper_feedback_data.joint_data.pitch = (upper_pitch_motor->measure.total_angle - upper_pitch_motor->measure.init_angle) * ROTOR_2_SHAFT_PITCH;
    upper_feedback_data.joint_data.roll_differ = (((upper_differ_motor_r->measure.total_angle - upper_differ_motor_r->measure.init_angle) - (-1.0f * upper_differ_motor_l->measure.total_angle + upper_differ_motor_l->measure.init_angle)) / 2.0f * GEAR_RATION_DIFFER) * ROTOR_2_SHAFT_ROLL_DIFFER;
    upper_feedback_data.joint_data.pitch_differ = (((upper_differ_motor_r->measure.total_angle - upper_differ_motor_r->measure.init_angle) + (-1.0f * upper_differ_motor_l->measure.total_angle + upper_differ_motor_l->measure.init_angle)) / 2.0f) * ROTOR_2_SHAFT_PITCH_DIFFER; // 电机反装，测量值需要取反

    upper_feedback_data.joint_data.lift_dist = ((upper_lift_motor_l->measure.total_angle - upper_lift_motor_l->measure.init_angle) - (upper_lift_motor_r->measure.total_angle - upper_lift_motor_r->measure.init_angle)) / 2.0f / LIFT_DIST_2_ANGLE;
    upper_feedback_data.joint_data.push_dist = ((upper_push_motor_l->measure.total_angle - upper_push_motor_l->measure.init_angle) - (upper_push_motor_r->measure.total_angle - upper_push_motor_r->measure.init_angle)) / 2.0f / PUSH_DIST_2_ANGLE;
    upper_feedback_data.joint_data.traverse_dist = -1.0f * (upper_traverse_motor->measure.total_angle - upper_traverse_motor->measure.init_angle) / TRAVERSE_DIST_2_ANGLE;

    upper_feedback_data.action_step = action_step;

    upper_last_mode = upper_cmd_recv.upper_mode;
}

/**
 * @brief 上层机构电机使能
 *
 */
static void UpperIMotorEnable()
{
    DJIMotorEnable(upper_yaw_motor);
    DJIMotorEnable(upper_roll_motor);
    DJIMotorEnable(upper_pitch_motor);
    DJIMotorEnable(upper_differ_motor_l);
    DJIMotorEnable(upper_differ_motor_r);
    DJIMotorEnable(upper_lift_motor_l);
    DJIMotorEnable(upper_lift_motor_r);
    DJIMotorEnable(upper_push_motor_l);
    DJIMotorEnable(upper_push_motor_r);
    DJIMotorEnable(upper_traverse_motor);
}

/**
 * @brief 将关节数据换算成电机输出数据
 *
 */
static void UpperCalculat()
{
    upper_yaw_op = upper_solve.yaw * SHAFT_2_ROTOR_YAW + upper_yaw_motor->measure.init_angle;
    upper_roll_op = upper_solve.roll * SHAFT_2_ROTOR_ROLL + upper_roll_motor->measure.init_angle;
    upper_pitch_op = upper_solve.pitch * SHAFT_2_ROTOR_PITCH + upper_pitch_motor->measure.init_angle;
    upper_differ_l_op = (upper_solve.pitch_differ - upper_solve.roll_differ / GEAR_RATION_DIFFER) * SHAFT_2_ROTOR_ROLL_DIFFER - upper_differ_motor_l->measure.init_angle; // 电机反装，测量的初始值需要取反
    upper_differ_r_op = (upper_solve.pitch_differ + upper_solve.roll_differ / GEAR_RATION_DIFFER) * SHAFT_2_ROTOR_ROLL_DIFFER + upper_differ_motor_r->measure.init_angle;
    upper_lift_l_op = upper_solve.lift_dist * LIFT_DIST_2_ANGLE + upper_lift_motor_l->measure.init_angle;
    upper_lift_r_op = upper_solve.lift_dist * LIFT_DIST_2_ANGLE - upper_lift_motor_r->measure.init_angle; // 电机反装，测量的初始值需要取反
    upper_push_l_op = upper_solve.push_dist * PUSH_DIST_2_ANGLE + upper_push_motor_l->measure.init_angle;
    upper_push_r_op = upper_solve.push_dist * PUSH_DIST_2_ANGLE - upper_push_motor_r->measure.init_angle;             // 电机反装，测量的初始值需要取反
    upper_traverse_op = upper_solve.traverse_dist * TRAVERSE_DIST_2_ANGLE - upper_traverse_motor->measure.init_angle; // 电机反装，测量的初始值需要取反
}

/**
 * @brief 上层机构无力模式
 *
 */
static void UpperZeroForceMode()
{
    DJIMotorStop(upper_yaw_motor);
    DJIMotorStop(upper_roll_motor);
    DJIMotorStop(upper_pitch_motor);
    DJIMotorStop(upper_differ_motor_l);
    DJIMotorStop(upper_differ_motor_r);
    DJIMotorStop(upper_lift_motor_l);
    DJIMotorStop(upper_lift_motor_r);
    DJIMotorStop(upper_push_motor_l);
    DJIMotorStop(upper_push_motor_r);
    DJIMotorStop(upper_traverse_motor);
    upper_lift_motor_l->measure.init_flag = 1;
    upper_lift_motor_r->measure.init_flag = 1;
}

/**
 * @brief 上层机构校准模式
 *
 */
static void UpperCaliMode()
{
    static uint16_t cali_time = 0;
    static const float speed_maxout_differ = 15000;
    static const float speed_maxout_pitch = 15000;
    static const float angle_maxout_pitch = 10000;
    static const float speed_maxout_roll = 15000;
    static const float speed_maxout_traverse = 15000;
    static const float speed_maxout_push = 15000;
    static const float speed_maxout_lift = 15000;
    static float pitch_test_max;
    static float pitch_test_min;

    if (action_step == 1)
    {
        // calibrate pitch_differ max
        if (action_finish_flag == 0 && action_flag == 0)
        {
            action_flag = 1;
            UpperCalculat();
            DJIMotorOuterLoop(upper_differ_motor_l, SPEED_LOOP);
            DJIMotorOuterLoop(upper_differ_motor_r, SPEED_LOOP);
            upper_differ_motor_l->motor_controller.speed_PID.MaxOut = 4000;
            upper_differ_motor_r->motor_controller.speed_PID.MaxOut = 4000;
            upper_differ_l_op = 15000;
            upper_differ_r_op = 15000;
        }
        else if (action_finish_flag == 0 && action_flag == 2)
        {
            action_flag = 3;
            upper_differ_l_op = -15000;
            upper_differ_r_op = -15000;
        }
        else if (action_flag == 0)
        {
            action_flag = 1;

            DJIMotorOuterLoop(upper_differ_motor_l, ANGLE_LOOP);
            DJIMotorOuterLoop(upper_differ_motor_r, ANGLE_LOOP);
            upper_solve.pitch_differ = upper_feedback_data.joint_data.pitch_differ + (pitch_test_max - pitch_test_min) / 2.0f - 10.0f;
            UpperCalculat();
        }

        if (fabsf(upper_differ_motor_l->measure.speed_aps) < 1.0f && fabsf(upper_differ_motor_r->measure.speed_aps) < 1.0f)
        {
            cali_time++;

            if (cali_time > CALI_STEP_TIME)
            {
                if (action_finish_flag == 0)
                {
                    cali_time = 0;
                    if (action_flag == 1)
                    {
                        action_flag = 2;
                        UpperCalculat();
                        pitch_test_max = upper_feedback_data.joint_data.pitch_differ;
                    }
                    else if (action_flag == 3)
                    {
                        action_flag = 0;
                        action_finish_flag = 1;
                        UpperCalculat();
                        pitch_test_min = upper_feedback_data.joint_data.pitch_differ;
                    }
                }
                else
                {
                    cali_time = 0;
                    action_flag = 0;
                    action_finish_flag = 0;
                    action_step++;
                    upper_differ_motor_l->measure.init_flag = 1;
                    upper_differ_motor_r->measure.init_flag = 1;
                    upper_solve.pitch_differ = 0;
                }
            }
        }
    }
    else if (action_step == 2)
    {
        // calibrate pitch max
        if (action_finish_flag == 0 && action_flag == 0)
        {
            action_flag = 1;
            UpperCalculat();
            DJIMotorOuterLoop(upper_pitch_motor, SPEED_LOOP);
            upper_pitch_motor->motor_controller.speed_PID.MaxOut = 3000;
            upper_pitch_op = 5000;
        }
        else if (action_flag == 0)
        {
            action_flag = 1;
            DJIMotorOuterLoop(upper_pitch_motor, ANGLE_LOOP);
            upper_pitch_motor->motor_controller.angle_PID.MaxOut = 4000; // 防止回来过程中pitch运动太快
            upper_solve.pitch = upper_feedback_data.joint_data.pitch - 20;
            UpperCalculat();
        }

        if (fabsf(upper_pitch_motor->measure.speed_aps) < EPS)
        {
            cali_time++;

            if (cali_time > CALI_STEP_TIME)
            {
                if (action_finish_flag == 0)
                {
                    cali_time = 0;
                    action_flag = 0;
                    action_finish_flag = 1;
                }
                else
                {
                    cali_time = 0;
                    action_flag = 0;
                    action_finish_flag = 0;
                    action_step++;
                    upper_pitch_motor->measure.init_flag = 1;
                    upper_solve.pitch = 0;
                    // UpperCalculat(); // 此时init_flag还未被置位，调用UpperCalculat函数会使pitch轴回到未校准时的位置
                }
            }
        }
    }
    else if (action_step == 3)
    {
        // calibrate roll max
        if (action_finish_flag == 0 && action_flag == 0)
        {
            action_flag = 1;
            UpperCalculat();
            DJIMotorOuterLoop(upper_roll_motor, SPEED_LOOP);
            upper_roll_motor->motor_controller.speed_PID.MaxOut = 3000;
            upper_roll_op = 15000;
        }
        else if (action_flag == 0)
        {
            action_flag = 1;
            DJIMotorOuterLoop(upper_roll_motor, ANGLE_LOOP);

            upper_solve.roll = upper_feedback_data.joint_data.roll - 120;
            UpperCalculat();
        }

        if (fabsf(upper_roll_motor->measure.speed_aps) < EPS)
        {
            cali_time++;

            if (cali_time > CALI_STEP_TIME)
            {
                if (action_finish_flag == 0)
                {
                    cali_time = 0;
                    action_flag = 0;
                    action_finish_flag = 1;
                }
                else
                {
                    cali_time = 0;
                    action_flag = 0;
                    action_finish_flag = 0;
                    action_step++;
                    upper_roll_motor->measure.init_flag = 1;
                    upper_solve.roll = 0;
                    // UpperCalculat();
                }
            }
        }
    }
    else if (action_step == 4)
    {
        action_step++;
        upper_yaw_motor->measure.init_flag = 1;
        upper_solve.yaw = 0;
    }
    else if (action_step == 5)
    {
        // calibrate traverse min
        if (action_finish_flag == 0 && action_flag == 0)
        {
            action_flag = 1;
            UpperCalculat();
            DJIMotorOuterLoop(upper_traverse_motor, SPEED_LOOP);
            upper_traverse_motor->motor_controller.speed_PID.MaxOut = 3000;
            upper_traverse_op = -15000;
        }

        if (fabsf(upper_traverse_motor->measure.speed_aps) < EPS)
        {
            cali_time++;

            if (cali_time > CALI_STEP_TIME)
            {
                cali_time = 0;
                action_flag = 0;
                action_step++;
                upper_traverse_motor->measure.init_flag = 1;
                DJIMotorOuterLoop(upper_traverse_motor, ANGLE_LOOP);
                upper_solve.traverse_dist = 0;
            }
        }
    }
    else if (action_step == 6)
    {
        // calibrate push min
        if (action_finish_flag == 0 && action_flag == 0)
        {
            action_flag = 1;
            UpperCalculat();
            DJIMotorOuterLoop(upper_push_motor_l, SPEED_LOOP);
            DJIMotorOuterLoop(upper_push_motor_r, SPEED_LOOP);
            upper_push_motor_l->motor_controller.speed_PID.MaxOut = 3000;
            upper_push_motor_r->motor_controller.speed_PID.MaxOut = 3000;
            upper_push_l_op = -15000;
            upper_push_r_op = -15000;
        }

        if (fabsf(upper_push_motor_l->measure.speed_aps) < EPS && fabsf(upper_push_motor_r->measure.speed_aps) < EPS)
        {
            cali_time++;

            if (cali_time > CALI_STEP_TIME)
            {
                cali_time = 0;
                action_flag = 0;
                action_step++;
                upper_push_motor_l->measure.init_flag = 1;
                upper_push_motor_r->measure.init_flag = 1;
                DJIMotorOuterLoop(upper_push_motor_l, ANGLE_LOOP);
                DJIMotorOuterLoop(upper_push_motor_r, ANGLE_LOOP);
                upper_solve.push_dist = 0;
            }
        }
    }
    else if (action_step == 7)
    {
        // calibrate lift min
        if (action_finish_flag == 0 && action_flag == 0)
        {
            action_flag = 1;
            UpperCalculat();
            DJIMotorOuterLoop(upper_lift_motor_l, SPEED_LOOP);
            DJIMotorOuterLoop(upper_lift_motor_r, SPEED_LOOP);
            upper_lift_motor_l->motor_controller.speed_PID.MaxOut = 1500;
            upper_lift_motor_r->motor_controller.speed_PID.MaxOut = 1500;
            upper_lift_l_op = -15000;
            upper_lift_r_op = -15000;
        }

        if (fabsf(upper_lift_motor_l->measure.speed_aps) < 100 && fabsf(upper_lift_motor_r->measure.speed_aps) < 100)
        {
            cali_time++;

            if (cali_time > CALI_STEP_TIME)
            {
                cali_time = 0;
                action_flag = 0;
                action_step++;
                DJIMotorStop(upper_lift_motor_l);
                DJIMotorStop(upper_lift_motor_r);
                DWT_Delay(1);
                upper_lift_motor_l->measure.init_flag = 1;
                upper_lift_motor_r->measure.init_flag = 1;
                DJIMotorOuterLoop(upper_lift_motor_l, ANGLE_LOOP);
                DJIMotorOuterLoop(upper_lift_motor_r, ANGLE_LOOP);
                upper_solve.lift_dist = 0;
            }
        }
    }
    else if (action_step == 8)
    {
        action_step = 0;
        upper_differ_motor_l->motor_controller.speed_PID.MaxOut = speed_maxout_differ;
        upper_differ_motor_r->motor_controller.speed_PID.MaxOut = speed_maxout_differ;
        upper_pitch_motor->motor_controller.speed_PID.MaxOut = speed_maxout_pitch;
        upper_pitch_motor->motor_controller.angle_PID.MaxOut = angle_maxout_pitch;
        upper_roll_motor->motor_controller.speed_PID.MaxOut = speed_maxout_roll;
        upper_traverse_motor->motor_controller.speed_PID.MaxOut = speed_maxout_traverse;
        upper_push_motor_l->motor_controller.speed_PID.MaxOut = speed_maxout_push;
        upper_push_motor_r->motor_controller.speed_PID.MaxOut = speed_maxout_push;
        upper_lift_motor_l->motor_controller.speed_PID.MaxOut = speed_maxout_push;
        upper_lift_motor_r->motor_controller.speed_PID.MaxOut = speed_maxout_push;
        UpperCalculat();
    }
}

/**
 * @brief 小资源岛开采银矿模式
 *
 */
static void UpperSliverMiningMode()
{
    static uint16_t cali_time = 0;

    switch (action_step)
    {
    case 1:
        // 第一步 展开机械臂
        upper_solve.lift_dist = 288.5f;
        upper_solve.push_dist = 67.0f;
        upper_solve.traverse_dist = TRAVERSE_MAX_DIST / 2.0f;
        upper_solve.yaw = 0;
        upper_solve.pitch = 0;
        upper_solve.roll = 0;
        upper_solve.pitch_differ = -90;
        break;
    case 2:
        upper_solve.pitch_differ = upper_cmd_recv.joint_data.pitch_differ;
        if (upper_cmd_recv.cfm_flag == 1)
            // 第二步 抓取矿石：lift向下、吸住矿石
            upper_solve.lift_dist = 230.0f;
        break;
    case 3:
        // 第三步：吸稳矿石后升起
        upper_solve.lift_dist = 450.0f;
        break;
    default:
        action_step = 0;
        break;
    }

    if (upper_cmd_recv.stop_flag == 0)
    {
        if ((action_step != 0) && (action_step != 2))
        {
            ActionFinishJudge(action_step, cali_time, 1, ACTION_STEP_TIME);
        }
        else if ((action_step == 2) && (upper_cmd_recv.cfm_flag))
        {
            ActionFinishJudge(action_step, cali_time, 1, CALI_STEP_TIME);
        }
    }
    else
    {
        ActionFinishJudge(action_step, cali_time, 0, ACTION_STEP_TIME);
    }
}

/**
 * @brief 小资源岛一键开采三个银矿模式
 *
 */
static void UpperSliverMiningThreeMode()
{
    static uint16_t cali_time = 0;

    switch (action_step)
    {
    case 1:
        upper_push_motor_l->motor_controller.angle_PID.MaxOut = 6000;
        upper_push_motor_r->motor_controller.angle_PID.MaxOut = 6000;
        upper_lift_motor_l->motor_controller.angle_PID.MaxOut = 6000;
        upper_lift_motor_r->motor_controller.angle_PID.MaxOut = 6000;
        // 第一步 伸出前伸并且抬高抬升 机械臂平行于地面
        upper_solve.traverse_dist = 0;
        upper_solve.yaw = 90;
        upper_solve.pitch = 0;
        upper_solve.roll = 0;
        upper_solve.pitch_differ = 0;
        break;
    case 2:
        upper_solve.lift_dist = 353.0f;
        upper_solve.push_dist = 250.0f;
        break;
    case 3:
        // //手动lift向下、吸住矿石
        // upper_solve.lift_dist = upper_cmd_recv.joint_data.lift_dist;

        // 第三步 将ui对准后 按下f抓取矿石：lift向下、吸住矿石
        if (upper_cmd_recv.cfm_flag == 3)
            upper_solve.lift_dist = 300.0f;
        break;
    case 4:
        // 第四步 抬起抬升
        upper_solve.lift_dist = 460;
        break;
    case 5:
        if (upper_cmd_recv.cfm_flag == 1)
        {
            upper_solve.yaw = 0;
        }
        break;
    case 6:
        upper_solve.pitch = -88.0f;
        upper_solve.lift_dist = 10;
        break;
    default:
        action_step = 0;
        upper_push_motor_l->motor_controller.angle_PID.MaxOut = 12000;
        upper_push_motor_r->motor_controller.angle_PID.MaxOut = 12000;
        upper_lift_motor_l->motor_controller.angle_PID.MaxOut = 12000;
        upper_lift_motor_r->motor_controller.angle_PID.MaxOut = 12000;
        break;
    }

    if (upper_cmd_recv.stop_flag == 0)
    {
        if ((action_step != 0) && (action_step != 3) && (action_step != 5))
        {
            ActionFinishJudge(action_step, cali_time, 1, ACTION_STEP_TIME);
        }
        else if ((action_step == 3) && (upper_cmd_recv.cfm_flag == 3))
        {
            ActionFinishJudge(action_step, cali_time, 1, 150);
        }
        else if ((action_step == 5) && (upper_cmd_recv.cfm_flag == 1))
        {
            ActionFinishJudge(action_step, cali_time, 1, ACTION_STEP_TIME);
        }
    }
    else
    {
        ActionFinishJudge(action_step, cali_time, 0, ACTION_STEP_TIME);
    }
}

/**
 * @brief 一位三矿上的银矿吸到机械臂上模式
 * @note  可以使用同cail一样的堵转检测来实现一位三矿，可以不用测量小资源岛高度，更加方便
 */
static void UpperGetSliverMiningThreeMode()
{
    static uint16_t cali_time = 0;

    if (upper_cmd_recv.getsliver_flag % 3 == 0)
    {
        switch (action_step)
        {
        case 1:
            // 第一步 把机械臂转下去
            upper_solve.traverse_dist = 0;
            upper_solve.yaw = 0;
            upper_solve.roll = -15;
            break;
        case 2:
            upper_solve.pitch = -90;
            upper_solve.pitch_differ = -90;
            break;
        case 3:

            break;
        default:
            action_step = 0;
            break;
        }
    }
    else if (upper_cmd_recv.getsliver_flag % 3 == 1)
    {
        switch (action_step)
        {
        case 1:
            // 第一步 把机械臂转下去
            upper_solve.traverse_dist = TRAVERSE_MAX_DIST / 2.0f;
            upper_solve.yaw = 0;
            upper_solve.pitch_differ = -90;
            upper_solve.roll = 0;
            break;
        case 2:
            upper_solve.pitch = -90;
            break;
        case 3:

            break;
        default:
            action_step = 0;
            break;
        }
    }
    else if (upper_cmd_recv.getsliver_flag % 3 == 2)
    {
        switch (action_step)
        {
        case 1:
            // 第一步 把机械臂转下去
            upper_solve.traverse_dist = TRAVERSE_MAX_DIST;
            upper_solve.yaw = 0;
            upper_solve.pitch_differ = -90;
            upper_solve.roll = 15;
            break;
        case 2:
            upper_solve.pitch = -90;
            break;
        case 3:
            break;
        default:
            action_step = 0;
            break;
        }
    }

    if (upper_cmd_recv.stop_flag == 0)
    {
        if (action_step != 0)
        {
            ActionFinishJudge(action_step, cali_time, 1, ACTION_STEP_TIME);
        }
    }
    else
    {
        ActionFinishJudge(action_step, cali_time, 0, ACTION_STEP_TIME);
    }
}

/**
 * @brief 大资源岛开采金矿模式
 *
 */
static void UpperGlodMiningMode()
{
    upper_solve.yaw = upper_cmd_recv.joint_data.yaw;
    upper_solve.roll = upper_cmd_recv.ctrlr_data.pitch;
    upper_solve.pitch_differ = upper_cmd_recv.ctrlr_data.yaw * 0.85f + upper_cmd_recv.joint_data.pitch;
    upper_solve.pitch = upper_cmd_recv.ctrlr_data.yaw * 0.15f + upper_cmd_recv.joint_data.pitch_differ;
    upper_solve.roll_differ = upper_cmd_recv.ctrlr_data.roll;
    upper_solve.lift_dist = upper_cmd_recv.joint_data.lift_dist;
    upper_solve.push_dist = upper_cmd_recv.ctrlr_data.push_dist;
    upper_solve.traverse_dist = upper_cmd_recv.ctrlr_data.traverse_dist;

    // 机械臂软件限位
    UpperJointConstrain(&upper_solve);
}

/**
 * @brief 金矿衔接模式
 *
 */
static void UpperGetGlodMiningMode()
{
    static uint16_t cali_time = 0;

    switch (action_step)
    {
    case 1:
        upper_solve.pitch_differ = 90;
        upper_solve.lift_dist = (LIFT_MAX_DIST / 2) + 10.0f;
        upper_solve.push_dist = PUSH_MAX_DIST;
        upper_solve.traverse_dist = TRAVERSE_MAX_DIST / 2.0f;
        break;
    case 2:
        upper_solve.pitch = PITCH_MAX;
        upper_solve.yaw = 180;
        upper_solve.pitch_differ = -90;
        break;
    default:
        action_step = 0;
        break;
    }
    if (upper_cmd_recv.stop_flag == 0)
    {
        if (action_step != 0)
            ActionFinishJudge(action_step, cali_time, 1, 120);
    }
    else
    {
        ActionFinishJudge(action_step, cali_time, 0, 60);
    }
}

/**
 * @brief 拾取地面矿模式
 *
 */
static void UpperGroundMiningMode()
{
    static uint16_t cali_time = 0;

    switch (action_step)
    {
    case 1:
        // 第一步：展开机械臂
        upper_solve.lift_dist = 130.0f;
        upper_solve.push_dist = 100.0f;
        upper_solve.traverse_dist = TRAVERSE_MAX_DIST / 2;
        upper_solve.yaw = 0;
        upper_solve.pitch = -90;
        upper_solve.roll = 0;
        upper_solve.pitch_differ = 0;
        break;
    case 2:
        upper_solve.pitch_differ = upper_cmd_recv.joint_data.pitch_differ;
        if (upper_cmd_recv.cfm_flag == 1)
            // 第二步 抓取矿石：lift向下、吸住矿石
            upper_solve.lift_dist = 60.0f;
        break;
    case 3:
        // 第三步：吸稳矿石后升起
        upper_solve.lift_dist = 130.0f;
        break;
    default:
        action_step = 0;
        break;
    }
    if (upper_cmd_recv.stop_flag == 0)
    {
        if (((action_step != 0) && (action_step != 2)) || ((action_step == 2) && (upper_cmd_recv.cfm_flag)))
            ActionFinishJudge(action_step, cali_time, 1, 150);
    }
    else
    {
        ActionFinishJudge(action_step, cali_time, 0, 60);
    }
}

/**
 * @brief 自定义控制器兑矿模式
 *
 */
static void UpperExchangeMode()
{
#ifdef EXCHANGE_SOLVE
    Upper_Kine.ctrlr_data.pitch = upper_cmd_recv.joint_data.pitch * DEGREE_2_RAD;
    Upper_Kine.ctrlr_data.yaw = -upper_cmd_recv.joint_data.yaw * DEGREE_2_RAD;
    Upper_Kine.ctrlr_data.roll = 0.0f;
    // Upper_Kine.ctrlr_data.roll = upper_cmd_recv.joint_data.roll * DEGREE_2_RAD;
    Upper_Kine.ctrlr_data.x = upper_cmd_recv.joint_data.traverse_dist;
    Upper_Kine.ctrlr_data.y = upper_cmd_recv.joint_data.push_dist + 397.44f;
    Upper_Kine.ctrlr_data.z = upper_cmd_recv.joint_data.lift_dist + 58.0f;

    // 解算出期望末端位姿：将欧拉角转换成齐次变换矩阵
    Upper_Kine.T_goal[0] = arm_cos_f32(Upper_Kine.ctrlr_data.yaw) * arm_sin_f32(Upper_Kine.ctrlr_data.roll);
    Upper_Kine.T_goal[1] = arm_cos_f32(Upper_Kine.ctrlr_data.yaw) * arm_cos_f32(Upper_Kine.ctrlr_data.roll);
    Upper_Kine.T_goal[2] = -arm_sin_f32(Upper_Kine.ctrlr_data.yaw);
    Upper_Kine.T_goal[3] = Upper_Kine.ctrlr_data.x;
    Upper_Kine.T_goal[4] = arm_cos_f32(Upper_Kine.ctrlr_data.pitch) * arm_sin_f32(Upper_Kine.ctrlr_data.yaw) * arm_sin_f32(Upper_Kine.ctrlr_data.roll) - arm_sin_f32(Upper_Kine.ctrlr_data.pitch) * arm_cos_f32(Upper_Kine.ctrlr_data.roll);
    Upper_Kine.T_goal[5] = arm_cos_f32(Upper_Kine.ctrlr_data.pitch) * arm_sin_f32(Upper_Kine.ctrlr_data.yaw) * arm_cos_f32(Upper_Kine.ctrlr_data.roll) + arm_sin_f32(Upper_Kine.ctrlr_data.pitch) * arm_sin_f32(Upper_Kine.ctrlr_data.roll);
    Upper_Kine.T_goal[6] = arm_cos_f32(Upper_Kine.ctrlr_data.pitch) * arm_cos_f32(Upper_Kine.ctrlr_data.yaw);
    Upper_Kine.T_goal[7] = Upper_Kine.ctrlr_data.y;
    Upper_Kine.T_goal[8] = arm_sin_f32(Upper_Kine.ctrlr_data.pitch) * arm_sin_f32(Upper_Kine.ctrlr_data.yaw) * arm_sin_f32(Upper_Kine.ctrlr_data.roll) + arm_cos_f32(Upper_Kine.ctrlr_data.pitch) * arm_cos_f32(Upper_Kine.ctrlr_data.roll);
    Upper_Kine.T_goal[9] = arm_sin_f32(Upper_Kine.ctrlr_data.pitch) * arm_sin_f32(Upper_Kine.ctrlr_data.yaw) * arm_cos_f32(Upper_Kine.ctrlr_data.roll) - arm_cos_f32(Upper_Kine.ctrlr_data.pitch) * arm_sin_f32(Upper_Kine.ctrlr_data.roll);
    Upper_Kine.T_goal[10] = arm_sin_f32(Upper_Kine.ctrlr_data.pitch) * arm_cos_f32(Upper_Kine.ctrlr_data.yaw);
    Upper_Kine.T_goal[11] = Upper_Kine.ctrlr_data.z;

    static float yaw, roll, pitch, x, y, z;
    float delta, last_delta;
    Slove_Type_e overflow_type = SOLVE_NO_OVERFLOW;
    Upper_Kine.calc_type = CALC_DOF_3;
    last_delta = 10000.0f;
    Upper_Kine.level = SOLVE_ALL_OVERFLOW;

    for (uint8_t i = 0; i < 2; i++)
    {
        if (i == 0)
        {
            roll = asinf(-Upper_Kine.T_goal[9]);
        }
        else
        {
            if (roll > EPS)
                roll = PI - roll;
            else if (roll < -EPS)
                roll = -PI - roll;
        }
        arm_atan2_f32(Upper_Kine.T_goal[5], Upper_Kine.T_goal[1], &yaw);
        arm_atan2_f32(-Upper_Kine.T_goal[10], Upper_Kine.T_goal[8], &pitch);
        if (roll < THETA5_MIN || roll > THETA5_MAX || yaw < THETA4_MIN || yaw > THETA4_MAX || pitch < THETA7_MIN || pitch > THETA7_MAX)
        {
            overflow_type = SOLVE_ALL_OVERFLOW;
        }

        x = Upper_Kine.T_goal[3] - ((-arm_cos_f32(yaw) * arm_sin_f32(roll) * arm_sin_f32(pitch) - arm_sin_f32(yaw) * arm_cos_f32(pitch)) * L7 - arm_sin_f32(yaw) * (L5 + L6));
        y = Upper_Kine.T_goal[7] - ((-arm_sin_f32(yaw) * arm_sin_f32(roll) * arm_sin_f32(pitch) + arm_cos_f32(yaw) * arm_cos_f32(pitch)) * L7 + arm_cos_f32(yaw) * (L5 + L6));
        z = Upper_Kine.T_goal[11] - (-arm_cos_f32(roll) * arm_sin_f32(pitch) * L7 + L4);

        if (overflow_type == SOLVE_NO_OVERFLOW)
            if (x < -EPS || x > TRAVERSE_MAX_SAFE_DIST || y < -EPS || y > PUSH_MAX_SAFE_DIST || z < -EPS || z > TRAVERSE_MAX_SAFE_DIST)
                overflow_type = SOLVE_POS_OVERFLOW;

        Upper_Kine.solve[i].joint_data.lift_dist = z;
        Upper_Kine.solve[i].joint_data.push_dist = y;
        Upper_Kine.solve[i].joint_data.traverse_dist = x;
        Upper_Kine.solve[i].joint_data.yaw = -yaw * RAD_2_DEGREE;
        Upper_Kine.solve[i].joint_data.roll = roll * RAD_2_DEGREE;
        Upper_Kine.solve[i].joint_data.pitch = 0;
        Upper_Kine.solve[i].joint_data.pitch_differ = -pitch * RAD_2_DEGREE;
        Upper_Kine.solve[i].joint_data.roll_differ = 0;
        Upper_Kine.solve[i].type = overflow_type;
        if (Upper_Kine.solve[i].type < Upper_Kine.level)
            Upper_Kine.level = Upper_Kine.solve[i].type;
    }

    if (Upper_Kine.level == SOLVE_ALL_OVERFLOW)
    {
        // 逆运动学解算出8组解
        static float d1, d2, d3,
            theta4, sin_theta4, cos_theta4,
            theta4_a_phi, sin_theta4_a_phi, cos_theta4_a_phi,
            theta5, sin_theta5, cos_theta5,
            theta6, sin_theta6, cos_theta6,
            theta7, sin_theta7, cos_theta7,
            theta6_a_theta7, sin_theta6_a_theta7, cos_theta6_a_theta7,
            theta8, sin_theta8, cos_theta8;
        static float A, B, C, rho, phi; // 中间变量
        overflow_type = SOLVE_NO_OVERFLOW;
        Upper_Kine.level = SOLVE_ALL_OVERFLOW;
        Upper_Kine.calc_type = CALC_DOF_5;

        // tehta6
        A = Upper_Kine.T_goal[3] - Upper_Kine.T_goal[2] * L7;
        B = Upper_Kine.T_goal[7] - Upper_Kine.T_goal[6] * L7;
        C = Upper_Kine.T_goal[11] - L4 - Upper_Kine.T_goal[10] * L7;
        cos_theta6 = (A * A + B * B + C * C - L5 * L5 - L6 * L6) / (2 * L5 * L6);
        if (cos_theta6 > 1.0f)
            cos_theta6 = 1.0f;
        for (uint8_t i = 0; i < 2; i++)
        {
            if (fabsf(1 - fabsf(cos_theta6)) < EPS)
                sin_theta6 = 0;
            else if (i == 0)
                sin_theta6 = arm_sqrt_f32(1 - cos_theta6 * cos_theta6, &sin_theta6);
            else
                sin_theta6 = -sin_theta6;
            arm_atan2_f32(sin_theta6, cos_theta6, &theta6);
            if ((theta6 < THETA6_MIN) || (theta6 > THETA6_MAX)) // 判断是否超过机械限位
            {
                overflow_type = SOLVE_ALL_OVERFLOW;
            }

            // theta4
            arm_sqrt_f32(A * A + B * B, &rho);
            arm_atan2_f32(A, B, &phi);
            cos_theta4_a_phi = (cos_theta6 * L6 + L5) / rho;
            for (uint8_t j = 0; j < 2; j++)
            {
                if (fabsf(1 - fabsf(cos_theta4_a_phi)) < EPS)
                    sin_theta4_a_phi = 0;
                else if (j == 0)
                    arm_sqrt_f32(1 - cos_theta4_a_phi * cos_theta4_a_phi, &sin_theta4_a_phi);
                else
                    sin_theta4_a_phi = -sin_theta4_a_phi;
                arm_atan2_f32(sin_theta4_a_phi, cos_theta4_a_phi, &theta4_a_phi);
                theta4 = theta4_a_phi - phi;
                if ((theta4 < THETA4_MIN) || (theta4 > THETA4_MAX))
                {
                    overflow_type = SOLVE_ALL_OVERFLOW;
                }
                sin_theta4 = arm_sin_f32(theta4);
                cos_theta4 = arm_cos_f32(theta4);

                // theta7
                cos_theta6_a_theta7 = -sin_theta4 * Upper_Kine.T_goal[2] + cos_theta4 * Upper_Kine.T_goal[6];
                for (uint8_t k = 0; k < 2; k++)
                {
                    if (fabsf(1 - fabsf(cos_theta6_a_theta7)) < EPS)
                        sin_theta6_a_theta7 = 0;
                    else if (k == 0)
                        arm_sqrt_f32(1 - cos_theta6_a_theta7 * cos_theta6_a_theta7, &sin_theta6_a_theta7);
                    else
                        sin_theta6_a_theta7 = -sin_theta6_a_theta7;
                    arm_atan2_f32(sin_theta6_a_theta7, cos_theta6_a_theta7, &theta6_a_theta7);
                    theta7 = theta6_a_theta7 - theta6;
                    if ((theta7 < THETA7_MIN) || (theta7 > THETA7_MAX))
                    {
                        overflow_type = SOLVE_ALL_OVERFLOW;
                    }

                    // theta5
                    if (fabsf(sin_theta6_a_theta7) > EPS)
                    {
                        sin_theta5 = (cos_theta4 * Upper_Kine.T_goal[2] + sin_theta4 * Upper_Kine.T_goal[6]) / (-sin_theta6_a_theta7);
                        cos_theta5 = Upper_Kine.T_goal[10] / (-sin_theta6_a_theta7);
                        arm_atan2_f32(sin_theta5, cos_theta5, &theta5);
                    }
                    if ((theta5 < THETA5_MIN) || (theta5 > THETA5_MAX))
                    {
                        overflow_type = SOLVE_ALL_OVERFLOW;
                    }

                    // theta8
                    sin_theta8 = Upper_Kine.T_goal[0] * cos_theta4 * cos_theta5 + Upper_Kine.T_goal[4] * sin_theta4 * cos_theta5 - Upper_Kine.T_goal[8] * sin_theta5;
                    cos_theta8 = Upper_Kine.T_goal[1] * cos_theta4 * cos_theta5 + Upper_Kine.T_goal[5] * sin_theta4 * cos_theta5 - Upper_Kine.T_goal[9] * sin_theta5;
                    arm_atan2_f32(sin_theta8, cos_theta8, &theta8);

                    // 得出移动关节
                    d3 = Upper_Kine.T_goal[3] -
                         (-cos_theta4 * sin_theta5 * sin_theta6 * L6 - sin_theta4 * cos_theta6 * L6 - sin_theta4 * L5 +
                          L7 * (-cos_theta4 * sin_theta5 * sin_theta6_a_theta7 - sin_theta4 * cos_theta6_a_theta7));
                    if ((d3 < -EPS) || (d3 > TRAVERSE_MAX_SAFE_DIST))
                    {
                        if (overflow_type == SOLVE_NO_OVERFLOW)
                            overflow_type = SOLVE_POS_OVERFLOW;
                    }
                    d2 = Upper_Kine.T_goal[7] -
                         (-sin_theta4 * sin_theta5 * sin_theta6 * L6 + cos_theta4 * cos_theta6 * L6 + cos_theta4 * L5 +
                          L7 * (-sin_theta4 * sin_theta5 * sin_theta6_a_theta7 + cos_theta4 * cos_theta6_a_theta7));
                    if ((d2 < -EPS) || (d2 > PUSH_MAX_SAFE_DIST))
                    {
                        if (overflow_type == SOLVE_NO_OVERFLOW)
                            overflow_type = SOLVE_POS_OVERFLOW;
                    }
                    d1 = Upper_Kine.T_goal[11] -
                         (-cos_theta5 * sin_theta6 * L6 + L4 +
                          L7 * (-cos_theta5 * sin_theta6_a_theta7));
                    if ((d1 < -EPS) || (d1 > LIFT_MAX_SAFE_DIST))
                    {
                        if (overflow_type == SOLVE_NO_OVERFLOW)
                            overflow_type = SOLVE_POS_OVERFLOW;
                    }

                    Upper_Kine.solve[4 * i + 2 * j + k].joint_data.lift_dist = d1;
                    Upper_Kine.solve[4 * i + 2 * j + k].joint_data.push_dist = d2;
                    Upper_Kine.solve[4 * i + 2 * j + k].joint_data.traverse_dist = d3;
                    Upper_Kine.solve[4 * i + 2 * j + k].joint_data.yaw = -theta4 * RAD_2_DEGREE;
                    Upper_Kine.solve[4 * i + 2 * j + k].joint_data.roll = theta5 * RAD_2_DEGREE;
                    Upper_Kine.solve[4 * i + 2 * j + k].joint_data.pitch = -theta6 * RAD_2_DEGREE;
                    Upper_Kine.solve[4 * i + 2 * j + k].joint_data.pitch_differ = -theta7 * RAD_2_DEGREE;
                    Upper_Kine.solve[4 * i + 2 * j + k].joint_data.roll_differ = theta8 * RAD_2_DEGREE;
                    Upper_Kine.solve[4 * i + 2 * j + k].type = overflow_type;
                    if (Upper_Kine.solve[4 * i + 2 * j + k].type < Upper_Kine.level)
                        Upper_Kine.level = Upper_Kine.solve[4 * i + 2 * j + k].type;
                }
            }
        }
    }

    for (uint8_t i = 0; i < Upper_Kine.calc_type; i++)
    {
        if (Upper_Kine.level == Upper_Kine.solve[i].type)
        {
            // 最优解选择
            delta = fabsf(Upper_Kine.solve[i].joint_data.lift_dist - upper_feedback_data.joint_data.lift_dist) +
                    fabsf(Upper_Kine.solve[i].joint_data.push_dist - upper_feedback_data.joint_data.push_dist) +
                    fabsf(Upper_Kine.solve[i].joint_data.traverse_dist - upper_feedback_data.joint_data.traverse_dist) +
                    100.0f * fabsf(Upper_Kine.solve[i].joint_data.yaw - upper_feedback_data.joint_data.yaw) +
                    100.0f * fabsf(Upper_Kine.solve[i].joint_data.roll - upper_feedback_data.joint_data.roll) +
                    100.0f * fabsf(Upper_Kine.solve[i].joint_data.pitch - upper_feedback_data.joint_data.pitch) +
                    100.0f * fabsf(Upper_Kine.solve[i].joint_data.pitch_differ - upper_feedback_data.joint_data.pitch_differ); // 计算解算值与当前真实关节位置的差值
            if (delta < last_delta)
            {
                Upper_Kine.output.lift_dist = Upper_Kine.solve[i].joint_data.lift_dist;
                Upper_Kine.output.push_dist = Upper_Kine.solve[i].joint_data.push_dist;
                Upper_Kine.output.traverse_dist = Upper_Kine.solve[i].joint_data.traverse_dist;
                Upper_Kine.output.yaw = Upper_Kine.solve[i].joint_data.yaw;
                Upper_Kine.output.roll = Upper_Kine.solve[i].joint_data.roll;
                Upper_Kine.output.pitch = Upper_Kine.solve[i].joint_data.pitch;
                Upper_Kine.output.pitch_differ = Upper_Kine.solve[i].joint_data.pitch_differ;
                Upper_Kine.output.roll_differ = Upper_Kine.solve[i].joint_data.roll_differ;
            }
            last_delta = delta;
        }
    }

    upper_solve.lift_dist = Upper_Kine.output.lift_dist;
    upper_solve.push_dist = Upper_Kine.output.push_dist;
    upper_solve.traverse_dist = Upper_Kine.output.traverse_dist;
    upper_solve.yaw = Upper_Kine.output.yaw;
    upper_solve.roll = Upper_Kine.output.roll;
    upper_solve.pitch = Upper_Kine.output.pitch;
    upper_solve.pitch_differ = Upper_Kine.output.pitch_differ;
    upper_solve.roll_differ = upper_cmd_recv.joint_data.roll_differ + upper_cmd_recv.joint_data.roll;
#endif // EXCHANGE_SOLVE
    // 相对角度控制
    /* upper_solve.yaw = upper_cmd_recv.joint_data.yaw;
    upper_solve.roll = upper_cmd_recv.ctrlr_data.pitch;
    upper_solve.pitch_differ = upper_cmd_recv.ctrlr_data.yaw + upper_cmd_recv.joint_data.pitch_differ;
    if ((upper_cmd_recv.joint_data.pitch_differ + upper_cmd_recv.ctrlr_data.yaw) - 90.0f > EPS)
        upper_solve.pitch = upper_cmd_recv.joint_data.pitch_differ + upper_cmd_recv.ctrlr_data.yaw - 90.0f;
    else if ((upper_cmd_recv.joint_data.pitch_differ + upper_cmd_recv.ctrlr_data.yaw) + 90.0f < -EPS)
        upper_solve.pitch = upper_cmd_recv.joint_data.pitch_differ + upper_cmd_recv.ctrlr_data.yaw + 90.0f;
    upper_solve.roll_differ = upper_cmd_recv.ctrlr_data.roll;
    upper_solve.lift_dist = upper_cmd_recv.joint_data.lift_dist;
    upper_solve.push_dist = upper_cmd_recv.joint_data.push_dist + upper_cmd_recv.ctrlr_data.push_dist;
    upper_solve.traverse_dist = upper_cmd_recv.joint_data.traverse_dist + upper_cmd_recv.ctrlr_data.traverse_dist; */

    // 绝对角度控制
    if (upper_last_mode != UPPER_EXCHANGE)
        upper_solve.pitch = 0;
    upper_solve.yaw = upper_cmd_recv.joint_data.yaw;
    upper_solve.roll = upper_cmd_recv.ctrlr_data.pitch;
    upper_solve.pitch_differ = upper_cmd_recv.ctrlr_data.yaw;
    if (upper_cmd_recv.ctrlr_data.yaw - 90.0f > EPS)
        upper_solve.pitch = upper_cmd_recv.ctrlr_data.yaw - 90.0f;
    else if (upper_cmd_recv.ctrlr_data.yaw + 90.0f < -EPS)
        upper_solve.pitch = upper_cmd_recv.ctrlr_data.yaw + 90.0f;
    upper_solve.roll_differ = upper_cmd_recv.ctrlr_data.roll;
    upper_solve.lift_dist = upper_cmd_recv.joint_data.lift_dist;
    upper_solve.push_dist = upper_cmd_recv.ctrlr_data.push_dist;
    upper_solve.traverse_dist = upper_cmd_recv.ctrlr_data.traverse_dist;

    // 机械臂软件限位
    UpperJointConstrain(&upper_solve);
}

/**
 * @brief 单轴控制模式
 *
 */
static void UpperSingleMode()
{
    upper_solve.yaw = upper_cmd_recv.joint_data.yaw;
    upper_solve.roll = upper_cmd_recv.joint_data.roll;
    upper_solve.pitch = upper_cmd_recv.joint_data.pitch;
    upper_solve.pitch_differ = upper_cmd_recv.joint_data.pitch_differ;
    upper_solve.roll_differ = upper_cmd_recv.joint_data.roll_differ;
    upper_solve.lift_dist = upper_cmd_recv.joint_data.lift_dist;
    upper_solve.push_dist = upper_cmd_recv.joint_data.push_dist;
    upper_solve.traverse_dist = upper_cmd_recv.joint_data.traverse_dist;
}

/**
 * @brief 根据模式选择控制方式
 *
 */
static void UpperModeControl()
{
    UpperIMotorEnable();
    if (upper_cmd_recv.upper_mode != upper_last_mode)
        action_step = 1;

    switch (upper_cmd_recv.upper_mode)
    {
    case UPPER_ZERO_FORCE:
        UpperZeroForceMode();
        break;
    case UPPER_NO_MOVE:
        break;
    case UPPER_CALI:
        UpperCaliMode();
        break;
    case UPPER_SINGLE_MOTOR:
        UpperSingleMode();
        break;
    case UPPER_SLIVER_MINING:
        UpperSliverMiningMode();
        break;
    case UPPER_THREE_SLIVER_MINING:
        UpperSliverMiningThreeMode();
        break;
    case UPPER_GET_THREE_SLIVER_MINING:
        UpperGetSliverMiningThreeMode();
        break;
    case UPPER_GLOD_MINING:
        UpperGlodMiningMode();
        break;
    case UPPER_GET_GLOD_MINING:
        UpperGetGlodMiningMode();
        break;
    case UPPER_GROUND_MINING:
        UpperGroundMiningMode();
        break;
    case UPPER_EXCHANGE:
        UpperExchangeMode();
        // UpperSingleMode();
        break;
    default:
        break;
    }
}

/**
 * @brief 设置电机参考值
 *
 */
static void UpperOutput()
{
    DJIMotorSetRef(upper_yaw_motor, upper_yaw_op);
    DJIMotorSetRef(upper_roll_motor, upper_roll_op);
    DJIMotorSetRef(upper_pitch_motor, upper_pitch_op);
    DJIMotorSetRef(upper_differ_motor_l, upper_differ_l_op);
    DJIMotorSetRef(upper_differ_motor_r, upper_differ_r_op);
    DJIMotorSetRef(upper_lift_motor_l, upper_lift_l_op);
    DJIMotorSetRef(upper_lift_motor_r, upper_lift_r_op);
    DJIMotorSetRef(upper_push_motor_l, upper_push_l_op);
    DJIMotorSetRef(upper_push_motor_r, upper_push_r_op);
    DJIMotorSetRef(upper_traverse_motor, upper_traverse_op);
}

void UpperTask()
{
    // 获取上层机构控制数据
    SubGetMessage(upper_sub, &upper_cmd_recv);

    // 根据控制模式设定上层机构工作方式
    UpperModeControl();

    // 将关节数据换算成电机输出数据
    if (upper_cmd_recv.upper_mode != UPPER_CALI)
        UpperCalculat();

    // 电机输出
    UpperOutput();

    // 更新反馈数据
    UpperFeedUpdata();

    // 反馈上层机构数据
    PubPushMessage(upper_pub, (void *)&upper_feedback_data);
}

/**
 * @brief 机械臂软件限位
 *
 */
void UpperJointConstrain(Upper_Joint_Data_s *joint_data)
{
    joint_data->yaw = float_constrain(joint_data->yaw, YAW_MIN, YAW_MAX);
    joint_data->roll = float_constrain(joint_data->roll, ROLL_MIN, ROLL_MAX);
    joint_data->pitch = float_constrain(joint_data->pitch, PITCH_MIN, PITCH_MAX);
    joint_data->pitch_differ = float_constrain(joint_data->pitch_differ, PITCH_DIFFER_MIN, PITCH_DIFFER_MAX);
    joint_data->lift_dist = float_constrain(joint_data->lift_dist, 0, LIFT_MAX_SAFE_DIST);
    joint_data->push_dist = float_constrain(joint_data->push_dist, 0, PUSH_MAX_SAFE_DIST);
    joint_data->traverse_dist = float_constrain(joint_data->traverse_dist, 0, TRAVERSE_MAX_SAFE_DIST);
}