/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310middleware.c/h
  * @brief      IST8310磁力计中间层，完成IST8310的IIC通信,因为设置MPU6500的SPI通信
  *             所以设置的是通过mpu6500的IIC_SLV0完成读取，IIC_SLV4完成写入。
  * @note       IST8310只支持IIC读取
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "ist8310driver_middleWare.h"
#include "stm32f4xx.h"
//#include "delay.h"

#include "mpu6500driver_middleWare.h"
#include "mpu6500reg.h"

void ist8310_com_init(void)
{
}

void ist8310_auto_com_by_mpu6500(void)
{
    uint8_t readBuf[3] = {IST8310_IIC_ADDRESS | IST8310_IIC_READ_MSB, 0x02, 0x88};
    mpu6500_write_muli_reg(MPU_I2CSLV0_ADDR, readBuf, 3);
}

uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    static const uint16_t IIC_time = 2000;
    uint8_t readBuf[3] = {IST8310_IIC_ADDRESS | IST8310_IIC_READ_MSB, reg, 0x81};
    mpu6500_write_muli_reg(MPU_I2CSLV0_ADDR, readBuf, 3);
    ist8310_delay_us(IIC_time);
    return mpu6500_read_single_reg(MPU_EXT_SENS_DATA_00);
}
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    uint8_t writeBuf[4] = {IST8310_IIC_ADDRESS, reg, data, 0x80};

    mpu6500_write_muli_reg(MPU_I2CSLV4_ADDR, writeBuf, 4);
}
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    while (len)
    {
        (*buf) = ist8310_IIC_read_single_reg(reg);
        reg++;
        buf++;
        len--;
    }
}
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    static const uint16_t IIC_time = 2000;
    while (len)
    {
        ist8310_IIC_write_single_reg(reg, (*data));
        reg++;
        data++;
        len--;
        ist8310_delay_us(IIC_time);
    }
}
void ist8310_delay_ms(uint16_t ms)
{
  HAL_Delay(ms);
}
void ist8310_delay_us(uint16_t us)
{
   HAL_Delay (us/1000);
}

void ist8310_RST_H(void)
{
	HAL_GPIO_WritePin (GPIOE,GPIO_PIN_2,GPIO_PIN_SET );
//    GPIO_SetBits(GPIOE, GPIO_Pin_2);
}
extern void ist8310_RST_L(void)
{
	HAL_GPIO_WritePin (GPIOE,GPIO_PIN_2,GPIO_PIN_RESET );
  //  GPIO_ResetBits(GPIOE, GPIO_Pin_2);
}
