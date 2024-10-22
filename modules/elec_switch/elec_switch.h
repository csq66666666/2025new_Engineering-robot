/*
 * @Descripttion: 电气开关模块定义头文件
 * @version: v1.0
 */
#ifndef ELEC_SWITCH_H
#define ELEC_SWITCH_H

#include "bsp_gpio.h"

#define ELECSWITCH_MAX_NUM 6   // 最大电气开关数量
#define VALVE1_Pin GPIO_PIN_12 // 电磁阀第1路 GPIO端口宏定义
#define VALVE1_GPIO_Port GPIOB
#define VALVE2_Pin GPIO_PIN_13 // 电磁阀第2路 GPIO端口宏定义
#define VALVE2_GPIO_Port GPIOB
#define VALVE3_Pin GPIO_PIN_14 // 电磁阀第3路 GPIO端口宏定义
#define VALVE3_GPIO_Port GPIOB
#define VALVE4_Pin GPIO_PIN_15 // 电磁阀第4路 GPIO端口宏定义
#define VALVE4_GPIO_Port GPIOB

#define HALL1_Pin GPIO_PIN_0 //霍尔开关1 GPIO端口定义 c板iic2_sda
#define HALL2_Pin GPIO_PIN_1 //霍尔开关2 GPIO端口定义 c板iic2_scl
#define HALL_GPIO_Port GPIOF

// 触发电平
typedef enum
{
    LOW = 0,
    HIGH,
} Trigger_Level_e;

/* 电气开关实例 */
typedef struct
{
    GPIOInstance *gpio_ins;        // GPIO实例
    Trigger_Level_e trigger_level; // 触发电平 GPIO_PIN_SET or GPIO_PIN_RESET
} ElecSwitchInstance;

/* 电气开关初始化配置 */
typedef struct
{
    GPIO_TypeDef *GPIOx;           // GPIOA,GPIOB,GPIOC...
    uint16_t GPIO_Pin;             // 引脚号,@note 这里的引脚号是GPIO_PIN_0,GPIO_PIN_1...
    uint32_t GPIO_Mode;            // 模式配置 （注意是hal_gpio定义的mode）
    Trigger_Level_e trigger_level; // 触发电平 HIGH or LOW
} ElecSwitch_Init_Config_s;

/**
 * @brief 初始化电气开关
 *
 * @param _config 电气开关初始化配置
 * @return ElecSwitchInstance* 电气开关实例指针
 */
ElecSwitchInstance *ElecSwitchInit(ElecSwitch_Init_Config_s *_config);

/**
 * @brief 电气开关置位
 *
 * @param _ins 电气开关实例指针
 */
void ElecSwitchSet(ElecSwitchInstance *_ins);

/**
 * @brief 电气开关复位
 *
 * @param _ins 电气开关实例指针
 */
void ElecSwitchReset(ElecSwitchInstance *_ins);

/**
 * @brief 电气开关读取
 *
 * @param _ins 电气开关实例指针
 */
GPIO_PinState ElecSwitchRead(ElecSwitchInstance *_ins);

#endif // !1#define