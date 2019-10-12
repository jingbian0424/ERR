#include "iic.h"



uint16_t b = 0;

//IIC��ʼ��
void IIC_Init(void)
{ 
    IIC_SDA_H;
    IIC_SCL_H;  
}

//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA_H;	  	  
	IIC_SCL_H;
	HAL_Delay(10);
	//
 	IIC_SDA_L;//START:when CLK is high,DATA change form high to low 
	HAL_Delay(10);
	IIC_SCL_L;//ǯסI2C���ߣ�׼�����ͻ�������� 
	HAL_Delay(10);
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL_L;
	IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
 	HAL_Delay(10);
	IIC_SCL_H; 
    HAL_Delay(10);		
	IIC_SDA_H;//����I2C���߽����ź�				   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
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
	IIC_SCL_L;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL_L;//����ʱ�ӿ�ʼ���ݴ���
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
	  
//		HAL_Delay(10);   //��TEA5767��������ʱ���Ǳ����
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
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
        IIC_NAck();//����nACK
	}
    else
	{
        IIC_Ack(); //����ACK  
	}	
    b =1; 	
    return receive;
}


