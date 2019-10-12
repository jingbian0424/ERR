#include "mpumiddleware.h"
#include "spi.h"
uint8_t RxDat;
void mpu6500_middleware_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}

void mpu6500_middleware_delay_us(uint32_t us)
{

      HAL_Delay(us/1000); 
}

void mpu6500_SPI_NS_H(void)
{
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6,GPIO_PIN_SET);    // GPIO_SetBits(GPIOF, GPIO_Pin_6);
}
void mpu6500_SPI_NS_L(void)
{
    HAL_GPIO_WritePin (GPIOF, GPIO_PIN_6,GPIO_PIN_RESET);  //GPIO_ResetBits(GPIOF, GPIO_Pin_6);
}

static uint8_t mpu6500_SPI_read_write_byte(uint8_t TxDat)
{
    uint8_t retry = 0;
    while ( HAL_SPI_Transmit_IT(&hspi5,&TxDat, 8)== HAL_OK )
    {
        retry++;
        if (retry > 200)
        {
            return 0;
        }
    }
    HAL_SPI_Transmit(&hspi5,&TxDat, 8, 10);
// SPI_I2S_SendData(SPI5, TxData);

    retry = 0;

    while (HAL_SPI_Receive_IT(&hspi5,&TxDat, 8) == HAL_OK )
    {
        retry++;
        if (retry > 200)
        {
            return 0;
        }
    }
    HAL_SPI_Receive (&hspi5,&RxDat, 8, 10);
    return HAL_SPI_Receive (&hspi5,&RxDat, 8, 10);
}
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
void mpu6500_write_single_reg(uint8_t reg, uint8_t data)
{
    mpu6500_SPI_NS_L();
    mpu6500_SPI_read_write_byte(reg);
    mpu6500_SPI_read_write_byte(data);
    mpu6500_SPI_NS_H();
}

uint8_t mpu6500_read_single_reg(uint8_t reg)
{
    uint8_t res;
    mpu6500_SPI_NS_L();
    mpu6500_SPI_read_write_byte(reg | MPU_SPI_READ_MSB);
    res = mpu6500_SPI_read_write_byte(0xFF);
    mpu6500_SPI_NS_H();
    return res;
}

void mpu6500_write_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    mpu6500_SPI_NS_L();
    mpu6500_SPI_read_write_byte(reg);
    if (len != 0)
    {
        uint8_t i;
        for (i = 0; i < len; i++)
        {
            mpu6500_SPI_read_write_byte(*buf);
            buf++;
        }
    }
    mpu6500_SPI_NS_H();
}

void mpu6500_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    mpu6500_SPI_NS_L();
    mpu6500_SPI_read_write_byte(reg | MPU_SPI_READ_MSB);
    if (len != 0)
    {
        uint8_t i;
        for (i = 0; i < len; i++)
        {
            *buf = mpu6500_SPI_read_write_byte(0xFF);
            buf++;
        }
    }
    mpu6500_SPI_NS_H();
}

















































































