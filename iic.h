#ifndef _IIC_H
#define _IIC_H


#include "main.h"
#include "stm32f4xx_hal.h"
//
#define SDA_IN()  {GPIOF->MODER&=~(3<<(5*2));GPIOF->MODER|=0<<5*2;}	//PH5����ģʽ
#define SDA_OUT() {GPIOF->MODER&=~(3<<(5*2));GPIOF->MODER|=1<<5*2;} //PH5���ģʽ
//
#define IIC_SCL_H    HAL_GPIO_WritePin (GPIOF,GPIO_PIN_7,GPIO_PIN_SET)
#define IIC_SCL_L    HAL_GPIO_WritePin (GPIOF,GPIO_PIN_7,GPIO_PIN_RESET)
//
#define IIC_SDA_H    HAL_GPIO_WritePin (GPIOF,GPIO_PIN_9,GPIO_PIN_SET)
#define IIC_SDA_L    HAL_GPIO_WritePin (GPIOF,GPIO_PIN_9,GPIO_PIN_RESET)
#define READ_SDA     HAL_GPIO_ReadPin (GPIOF,GPIO_PIN_9)
//
//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	 
#endif







