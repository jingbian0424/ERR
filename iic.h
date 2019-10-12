#ifndef _IIC_H
#define _IIC_H


#include "main.h"
#include "stm32f4xx_hal.h"
//
#define SDA_IN()  {GPIOF->MODER&=~(3<<(5*2));GPIOF->MODER|=0<<5*2;}	//PH5输入模式
#define SDA_OUT() {GPIOF->MODER&=~(3<<(5*2));GPIOF->MODER|=1<<5*2;} //PH5输出模式
//
#define IIC_SCL_H    HAL_GPIO_WritePin (GPIOF,GPIO_PIN_7,GPIO_PIN_SET)
#define IIC_SCL_L    HAL_GPIO_WritePin (GPIOF,GPIO_PIN_7,GPIO_PIN_RESET)
//
#define IIC_SDA_H    HAL_GPIO_WritePin (GPIOF,GPIO_PIN_9,GPIO_PIN_SET)
#define IIC_SDA_L    HAL_GPIO_WritePin (GPIOF,GPIO_PIN_9,GPIO_PIN_RESET)
#define READ_SDA     HAL_GPIO_ReadPin (GPIOF,GPIO_PIN_9)
//
//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	 
#endif







