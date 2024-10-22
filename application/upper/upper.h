#ifndef UPPER_H
#define UPPER_H

#include "general_def.h"
#include "user_lib.h"
#include "robot_def.h"

#define L4 58
#define L5 119
#define L6 227
#define L7 51.44
#define THETA4_MAX (PI / 2)
#define THETA4_MIN (-PI / 2)
#define THETA5_MAX (110.0f * DEGREE_2_RAD)
#define THETA5_MIN (-110.0f * DEGREE_2_RAD)
#define THETA6_MAX (90.0f * DEGREE_2_RAD)
#define THETA6_MIN (-20.0f * DEGREE_2_RAD)
#define THETA7_MAX (90.0f * DEGREE_2_RAD)
#define THETA7_MIN (-90.0f * DEGREE_2_RAD)

/* -------------------------私有数据类型-------------------------*/
typedef enum
{
    SOLVE_NO_OVERFLOW = 0, // 无溢出
    SOLVE_POS_OVERFLOW,    // 位置溢出
    SOLVE_ALL_OVERFLOW,    // 姿态溢出
} Slove_Type_e;

typedef enum
{
    CALC_DOF_3 = 2, // 使用三轴解算
    CALC_DOF_5 = 8, // 使用五轴解算
} Calc_Type_e;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
    float x;
    float y;
    float z;
} Upper_Ctrlr_s;

typedef struct
{
    Upper_Joint_Data_s joint_data;
    Slove_Type_e type;

} Upper_iKine_s;

typedef struct
{
    Upper_Ctrlr_s ctrlr_data; // 自定义控制器的数据 单位：角度
    float T_goal[16];         // 目标位姿的齐次变换矩阵
    float T_now[16];          // 当前位姿的齐次变换矩阵
    Upper_iKine_s solve[8];   // 运动学逆解得到的8组解 单位：弧度
    Slove_Type_e level;
    Calc_Type_e calc_type;
    Upper_Joint_Data_s output; // 最终输出解 单位：弧度
} Upper_Kine_s;

/**
 * @brief 上层应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 *
 */
void UpperInit();

/**
 * @brief 上层应用任务,放入实时系统以一定频率运行
 *
 */
void UpperTask();

/**
 * @brief 上层机构关节限位
 *
 */
void UpperJointConstrain(Upper_Joint_Data_s *joint_data);

#endif // UPPER_H