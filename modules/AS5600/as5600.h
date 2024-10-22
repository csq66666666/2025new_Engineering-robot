#ifndef __AS5600_H
#define __AS5600_H

// #include "sys.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
/******************************************************************************************/
/* 引脚 定义 */

#define IIC_SCL_GPIO_PORT               GPIOF
#define IIC_SCL_GPIO_PIN                GPIO_PIN_1
#define IIC_SCL_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

#define IIC_SDA_GPIO_PORT               GPIOF
#define IIC_SDA_GPIO_PIN                GPIO_PIN_0
#define IIC_SDA_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

#define	_raw_ang_hi 0x0c //as5600读值寄存器高8位地址
#define	_raw_ang_lo 0x0d //as5600读值寄存器低8位地址

/******************************************************************************************/
/* PCA9548A通道值 */

#define PCA9548A_CHANNEL_0          0x01
#define PCA9548A_CHANNEL_1          0x02
#define PCA9548A_CHANNEL_2          0x04
#define PCA9548A_CHANNEL_3          0x08
#define PCA9548A_CHANNEL_4          0x10
#define PCA9548A_CHANNEL_5          0x20
#define PCA9548A_CHANNEL_6          0x40
#define PCA9548A_CHANNEL_7          0x80
/******************************************************************************************/
/* IO操作 */

#define IIC_SCL(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SCL */

#define IIC_SDA(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SDA */

#define IIC_READ_SDA     HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN) /* 读取SDA */

/******************************************************************************************/   	   		   
//IO方向设置

#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式

/******************************************************************************************/
/* IIC所有操作函数 */

void iic_init(void);            /* 初始化IIC的IO口 */
void iic_start(void);           /* 发送IIC开始信号 */
void iic_stop(void);            /* 发送IIC停止信号 */
void iic_ack(void);             /* IIC发送ACK信号 */
void iic_nack(void);            /* IIC不发送ACK信号 */
uint8_t iic_wait_ack(void);     /* IIC等待ACK信号 */
void iic_send_byte(uint8_t txd);/* IIC发送一个字节 */
uint8_t iic_read_byte(unsigned char ack);/* IIC读取一个字节 */

// void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
// uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
 

/******************************************************************************************/
/* 读写控制寄存器 */

void PCA9548A_WriteOneByte(uint8_t WriteData);
uint8_t PCA9548A_ReadOneByte(void);

/******************************************************************************************/
/* 读写as5600传感器 */ //有两个 看情况用

uint8_t AS5600_ReadOneByte(uint16_t ReadAddr);
void AS5600_WriteOneByte(uint16_t WriteAddr,uint8_t WriteData);
uint16_t AS5600_ReadTwoByte(uint16_t ReadAddr_hi,uint16_t ReadAddr_lo);
int as5600_readangle(void);

uint16_t AS5600_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
float Get_Angle(void);

/* AS5600实例结构体定义 */
typedef struct
{
    int angle;

    
} AS5600Instance;

#endif
