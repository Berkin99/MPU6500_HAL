/*
 *	mpu6500.c
 *
 *  Created on: Fab 20, 2024
 *      Author: BerkN
 *
 *  mpu6500 spi driver library for stm32 microcontrollers.
 *  HAL Library integration.
 *
 *  20.02.2024 : Created.
 *
 *	References:
 *  [0] MPU-6500-Register-Map2.pdf
 * 	[1] PS-MPU-6500A-01-v1.3.pdf
 *
 */


#ifndef MPU6500_H_
#define MPU6500_H_

#include <stdint.h>
#include "system.h"

typedef enum {
	GYRO_RANGE_250DPS = 0,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS
} GyroRange;

typedef enum {
	ACCEL_RANGE_2G = 0,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
} AccelRange;

typedef enum {
	DLPF_BANDWIDTH_250HZ = 0,
	DLPF_BANDWIDTH_184HZ,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
} DLPFBandwidth;

typedef enum {
    SMPL_1000HZ = 0,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ,
} SampleRateDivider;

typedef enum{
	CLOCK_INTERNAL,
	CLOCK_PLL_XGYRO,
	CLOCK_PLL_YGYRO,
	CLOCK_PLL_ZGYRO,
	CLOCK_PLL_EXT32K,
	CLOCK_PLL_EXT19M,
	CLOCK_KEEP_RESET,
}ClockSource;

uint8_t MPU6500_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* gpio, uint16_t pin);
uint8_t MPU6500_Test(void);

void MPU6500_readRegister(uint8_t target, uint8_t* pRxBuffer, uint8_t length);
void MPU6500_writeRegister(uint8_t target, uint8_t* pData, uint8_t length);

uint8_t MPU6500_getDeviceID();
void MPU6500_GetData(int16_t* AccData, int16_t* GyroData);
void MPU6500_SetSleepMode(uint8_t isSleep);

void MPU6500_SetClockSource(ClockSource src);
void MPU6500_SetSampleRateDivider(SampleRateDivider srd);
void MPU6500_SetDLPFBandwidth(DLPFBandwidth bandwidth);
void MPU6500_SetGyroRange(GyroRange range);
void MPU6500_SetAccelRange(AccelRange range);

uint8_t MPU6500_GetClockSource(void);
uint8_t MPU6500_GetSampleRateDivider(void);
uint8_t MPU6500_GetDLPFBandwidth(void);
uint8_t MPU6500_GetGyroRange(void);
uint8_t MPU6500_GetAccelRange(void);

void MPU6500_SetAccelDLPFBandwidth(void);

#endif /* MPU6500_H_ */
