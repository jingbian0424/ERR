#include "iic.h"



uint16_t b = 0;

//IIC初始化
void IIC_Init(void)
{ 
    IIC_SDA_H;
    IIC_SCL_H;  
}

//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA_H;	  	  
	IIC_SCL_H;
	HAL_Delay(10);
	//
 	IIC_SDA_L;//START:when CLK is high,DATA change form high to low 
	HAL_Delay(10);
	IIC_SCL_L;//钳住I2C总线，准备发送或接收数据 
	HAL_Delay(10);
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL_L;
	IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
 	HAL_Delay(10);
	IIC_SCL_H; 
    HAL_Delay(10);		
	IIC_SDA_H;//发送I2C总线结束信号				   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA_H;HAL_Delay(10);
	IIC_SCL_H;HAL_Delay(10);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_L;
	HAL_Delay(10);
	IIC_SCL_H;
    HAL_Delay(10);
	IIC_SCL_L;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_H;
	HAL_Delay(10);
	IIC_SCL_H;
	HAL_Delay(10);
	IIC_SCL_L;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL_L;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {      
if(txd&0x80)  
{
	IIC_SDA_H ;	
}	
else
{
	IIC_SDA_L;
}
//         = (txd&0x80)>>7;
	  
//		HAL_Delay(10);   //对TEA5767这三个延时都是必须的
		IIC_SCL_H;
	    HAL_Delay(10);
		IIC_SCL_L;	
//		HAL_Delay(10);
if(t==7)
{
		IIC_SDA_H ;	
}
       txd<<=1; 
HAL_Delay (10);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL_L; 
        HAL_Delay(10);
		IIC_SCL_H;
        receive<<=1;
        if(READ_SDA)receive++;   
		HAL_Delay(10);
    }					 
    if (!ack)
	{
        IIC_NAck();//发送nACK
	}
    else
	{
        IIC_Ack(); //发送ACK  
	}	
    b =1; 	
    return receive;
}


