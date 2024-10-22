#include "as5600.h"
#include "gpio.h"
#include "bsp_dwt.h"

uint16_t raw_num = 0; //用来储存as5600读取的原始数据
uint16_t as5600angle = 0; //用来储存as5600读取的角度值

/**
 * @brief       初始化IIC
 * @param       无
 * @retval      无
 */
void iic_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    IIC_SCL_GPIO_CLK_ENABLE();  /* SCL引脚时钟使能 */
    IIC_SDA_GPIO_CLK_ENABLE();  /* SDA引脚时钟使能 */

    gpio_init_struct.Pin = IIC_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;      /* 高速 */
    HAL_GPIO_Init(IIC_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */
    gpio_init_struct.Pin = IIC_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;        /* 开漏输出 */
    HAL_GPIO_Init(IIC_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

    iic_stop();     /* 停止总线上所有设备 */
}

/**
 * @brief       IIC延时函数,用于控制IIC读写速度
 * @param       无
 * @retval      无
 */
static void iic_delay(void)
{
    DWT_Delay(0.000004);
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void iic_start(void)
{
    SDA_OUT();     //sda线输出
    IIC_SDA(1);
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(0);     
    iic_delay();
    IIC_SCL(0);     
}

/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void iic_stop(void)
{
    SDA_OUT();     //sda线输出
    IIC_SCL(0);
    IIC_SDA(0);     
    iic_delay();
    IIC_SCL(1);
    IIC_SDA(1);     
    iic_delay();
}

/**
 * @brief       等待应答信号到来
 * @param       无
 * @retval      1，接收应答失败
 *              0，接收应答成功
 */
uint8_t iic_wait_ack(void)
{
    uint8_t ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA(1);DWT_Delay(0.000001);	   
	IIC_SCL(1);DWT_Delay(0.000001);
	while(IIC_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			iic_stop();
			return 1;
		}
	}
	IIC_SCL(0); 	   
	return 0;  
}

/**
 * @brief       产生ACK应答
 * @param       无
 * @retval      无
 */
void iic_ack(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(0);     /* SCL 0 -> 1  时 SDA = 0,表示应答 */
    DWT_Delay(0.000002);
    IIC_SCL(1);     
    DWT_Delay(0.000002);
    IIC_SCL(0);
}

/**
 * @brief       不产生ACK应答
 * @param       无
 * @retval      无
 */
void iic_nack(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(1);     /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    DWT_Delay(0.000002);
    IIC_SCL(1);     
    DWT_Delay(0.000002);
    IIC_SCL(0);
}

/**
 * @brief       IIC发送一个字节
 * @param       data: 要发送的数据
 * @retval      无
 */
void iic_send_byte(uint8_t data)
{
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL(0);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA((data&0x80)>>7);
        data<<=1; 	  
		DWT_Delay(0.000002);   
		IIC_SCL(1);
		DWT_Delay(0.000002); 
		IIC_SCL(0);	
		DWT_Delay(0.000002);
    }	 
}

/**
 * @brief       IIC读取一个字节
 * @param       ack:  ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t iic_read_byte(uint8_t ack)
{
    unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++)
	{
        IIC_SCL(0); 
        DWT_Delay(0.000002);
		IIC_SCL(1);
        receive<<=1;
        if(IIC_READ_SDA)receive++;   
		DWT_Delay(0.000001); 
    }					 
    if (!ack)
        iic_nack();//发送nACK
    else
        iic_ack(); //发送ACK   
    return receive;
}

/******************************************************************************************/
/* 读写PCA9548A控制寄存器 */

/**
 * @brief PCA9548A写一个字节/写控制寄存器
 * @note  要让哪个通道打开就写哪个值
 * @param WriteData 通道值
 */
void PCA9548A_WriteOneByte(uint8_t WriteData)
{				  	  	    																 
    iic_start();
	iic_send_byte((0x70<<1)|0x00);
	iic_wait_ack();
	// iic_send_byte(WriteAddr); //只有一个寄存器 可以不用这三步
	// iic_wait_ack();
	// iic_start();
	iic_send_byte(WriteData);
	iic_wait_ack();
	iic_stop();
	DWT_Delay(0.000002);
}

//pca985读取控制寄存器
uint8_t PCA9548A_ReadOneByte(void)
{
	uint8_t temp_pca=-1;
	iic_start();  
	iic_send_byte((0x70<<1)|0x01);
	iic_wait_ack();
	temp_pca = iic_read_byte(0);
    iic_stop();    
	return temp_pca;
}

/******************************************************************************************/
/* 读写as5600传感器 */ //有两个 看情况用

/**
 * @brief 读取as5600传感器寄存器
 * @note  在Get_Angle();中使用
 * @param addr 从机地址 
 * @param reg  寄存器地址
 * @param len  读取长度
 * @param buf  读取数据缓存区
 */
uint16_t AS5600_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	//SDA_IN();
	
	iic_start();
	
	iic_send_byte((addr << 1) | 0);
	
	if (iic_wait_ack())
	{
		iic_stop();
		
		return 1;
	}
	
	iic_send_byte(reg);
	
	iic_wait_ack();
	
	iic_start();
	
	iic_send_byte((addr<<1) | 1);//发送器件地址+读命令
	
    iic_wait_ack();		//等待应答 
	
	while(len)
	{
		if(len==1)*buf=iic_read_byte(0);//读数据,发送nACK 
		else *buf=iic_read_byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
	
	iic_stop();	//产生一个停止条件 
	
	return 0;	
		
}

/**
 * @brief 获取as5600角度
 * @return （temp/4096*360） 角度值
 */
float Get_Angle(void)
{
	uint8_t buf[2] = {0};
	uint8_t i = 0;
	
	float temp = 0;
	float temp1 = 0.0;

	for (i = 0; i < 20; i++)
	{
	// AS5600_Read_Len(0x36,_raw_ang_hi,2,buf);
	AS5600_Read_Len(0x36,_raw_ang_hi,1,buf);
	AS5600_Read_Len(0x36,_raw_ang_lo,1,buf+1);
		temp1 +=buf[0]*256+buf[1];
		
	    DWT_Delay(0.005);
		//temp = (((u16)buf[0] & (0x0f00)) << 8) | buf[1];
	}
	
	
	//软件滤波，防止数据不稳定
	temp = temp1/20;
	
	return temp/4096*360;
}

//as5600读一个字节
uint8_t AS5600_ReadOneByte(uint16_t ReadAddr)
{				  
	uint8_t temp=-1;		  	    																 
    iic_start();  
	iic_send_byte((0X36<<1)|0x00);	   
	iic_wait_ack(); 
    iic_send_byte(ReadAddr);   
	iic_wait_ack();	    
	iic_start();  	 	   
	iic_send_byte((0X36<<1)|0x01);          	   
	iic_wait_ack();	 
    temp=iic_read_byte(0);		   
    iic_stop();    
	return temp;
}

//as5600读两个字节
uint16_t AS5600_ReadTwoByte(uint16_t ReadAddr_hi,uint16_t ReadAddr_lo)
{
	uint16_t TwoByte_Data=-1;
	uint8_t hi_Data=0,lo_Data=0;
	hi_Data=AS5600_ReadOneByte(ReadAddr_hi);
	lo_Data=AS5600_ReadOneByte(ReadAddr_lo);
	TwoByte_Data = (hi_Data<<8)|lo_Data;
	return TwoByte_Data;
}

//as5600写一个字节
void AS5600_WriteOneByte(uint16_t WriteAddr,uint8_t WriteData)
{				  	  	    																 
    iic_start();  
	iic_send_byte((0X36<<1)|0x00);
	iic_wait_ack(); 
    iic_send_byte(WriteAddr);
	iic_wait_ack();	    
	iic_start();  	 	   
	iic_send_byte(WriteData);	   
	iic_wait_ack();	 	   
    iic_stop();    
	DWT_Delay(0.000002);
}

//as5600角度读取
int as5600_readangle(void)
{
    raw_num = AS5600_ReadTwoByte(_raw_ang_hi,_raw_ang_lo);  //读取两个寄存器的值
    as5600angle = (raw_num*360)/4096; //对寄存器值进行处理得到角度值
    return as5600angle;
}