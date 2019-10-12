#ifndef _MPUM_H
#define _MPUM_H

#include "stm32f4xx_hal.h"


#define MPU6500_GYRO_RANGE_1000
#define MPU6500_ACCEL_RANGE_2G
//
extern void mpu6500_middleware_delay_ms(uint16_t ms);
extern void mpu6500_middleware_delay_us(uint32_t us);
#define MPU_SPI_READ_MSB 0x80

//������SPI NS �ߵ͵�ƽ
extern void mpu6500_SPI_NS_H(void);
extern void mpu6500_SPI_NS_L(void);

//�����Ƕ�ȡ��д��Ĵ�����ַ������
extern void mpu6500_write_single_reg(uint8_t reg, uint8_t data);
extern uint8_t mpu6500_read_single_reg(uint8_t reg);
extern void mpu6500_write_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void mpu6500_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);


#include "stm32f4xx_hal.h"
























































#endif 
