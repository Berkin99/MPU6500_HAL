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

#include "MPU6500.h"

#define REG_XG_OFFS_TC           0x00
#define REG_YG_OFFS_TC           0x01
#define REG_ZG_OFFS_TC           0x02
#define REG_X_FINE_GAIN          0x03
#define REG_Y_FINE_GAIN          0x04
#define REG_Z_FINE_GAIN          0x05
#define REG_XA_OFFS_H            0x06
#define REG_XA_OFFS_L            0x07
#define REG_YA_OFFS_H            0x08
#define REG_YA_OFFS_L            0x09
#define REG_ZA_OFFS_H            0x0A
#define REG_ZA_OFFS_L            0x0B
#define REG_PRODUCT_ID           0x0C
#define REG_SELF_TEST_X          0x0D
#define REG_SELF_TEST_Y          0x0E
#define REG_SELF_TEST_Z          0x0F
#define REG_SELF_TEST_A          0x10
#define REG_XG_OFFS_USRH         0x13
#define REG_XG_OFFS_USRL         0x14
#define REG_YG_OFFS_USRH         0x15
#define REG_YG_OFFS_USRL         0x16
#define REG_ZG_OFFS_USRH         0x17
#define REG_ZG_OFFS_USRL         0x18
#define REG_SMPLRT_DIV           0x19
#define REG_CONFIG               0x1A
#define REG_GYRO_CONFIG          0x1B
#define REG_ACCEL_CONFIG         0x1C
#define REG_INT_PIN_CFG          0x37
#define REG_INT_ENABLE           0x38
#define REG_ACCEL_XOUT_H         0x3B
#define REG_ACCEL_XOUT_L         0x3C
#define REG_ACCEL_YOUT_H         0x3D
#define REG_ACCEL_YOUT_L         0x3E
#define REG_ACCEL_ZOUT_H         0x3F
#define REG_ACCEL_ZOUT_L         0x40
#define REG_TEMP_OUT_H           0x41
#define REG_TEMP_OUT_L           0x42
#define REG_GYRO_XOUT_H          0x43
#define REG_GYRO_XOUT_L          0x44
#define REG_GYRO_YOUT_H          0x45
#define REG_GYRO_YOUT_L          0x46
#define REG_GYRO_ZOUT_H          0x47
#define REG_GYRO_ZOUT_L          0x48
#define REG_USER_CTRL            0x6A
#define REG_PWR_MGMT_1           0x6B
#define REG_PWR_MGMT_2           0x6C
#define REG_BANK_SEL             0x6D
#define REG_MEM_START_ADDR       0x6E
#define REG_MEM_R_W              0x6F
#define REG_DMP_CFG_1            0x70
#define REG_DMP_CFG_2            0x71
#define REG_FIFO_COUNTH          0x72
#define REG_FIFO_COUNTL          0x73
#define REG_FIFO_R_W             0x74
#define REG_WHOAMI               0x75

#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10

#define READ_FLAG                   0x80

SPI_HandleTypeDef* MPU6500_hspi;
GPIO_TypeDef* cs_gpio;
uint16_t cs_pin;

static uint8_t _buffer [14];

void activate(void){
	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_RESET);
}

void deactivate(void){
	HAL_GPIO_WritePin(cs_gpio, cs_pin, GPIO_PIN_SET);
}


uint8_t MPU6500_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* gpio, uint16_t pin){
	MPU6500_hspi = hspi;
	cs_gpio = gpio;
	cs_pin = pin;
	MPU6500_SetGyroRange(GYRO_RANGE_1000DPS);
	MPU6500_SetAccelRange(ACCEL_RANGE_8G);
	MPU6500_SetSleepMode(0);
	return 1;
}

uint8_t MPU6500_Test(void){
	if(MPU6500_getDeviceID() == 0x70) return 1;
	return 0;
}

void MPU6500_readRegister(uint8_t target, uint8_t* pRxBuffer, uint8_t length){
	activate();
	target |= READ_FLAG;
	HAL_SPI_Transmit(MPU6500_hspi, &target, 1, 50);
	HAL_SPI_Receive(MPU6500_hspi, pRxBuffer, length, 100);
	deactivate();
}


void MPU6500_writeRegister(uint8_t target, uint8_t* pData, uint8_t length){
	activate();
	HAL_SPI_Transmit(MPU6500_hspi, &target, 1, 50);
	HAL_SPI_Transmit(MPU6500_hspi, pData, length, 100);
	deactivate();
}

uint8_t MPU6500_getDeviceID(void){
	uint8_t buf = 0;
	MPU6500_readRegister(REG_WHOAMI, &buf, 1);
	return buf;
}

void MPU6500_SetSleepMode(uint8_t isSleep){
	uint8_t reg = 0;
	MPU6500_readRegister(REG_PWR_MGMT_1, &reg, 1);
	reg &= 0b10111111; 						// Clear the setting area
	reg |= ((isSleep & 0b00000001)<<6); 	// Add setting to config register value
	MPU6500_writeRegister(REG_CONFIG, &reg, 1);
}


void MPU6500_GetData(int16_t* AccData, int16_t* GyroData){
	MPU6500_readRegister(REG_ACCEL_XOUT_H,_buffer,14);

	AccData[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
	AccData[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
	AccData[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
	GyroData[0] = (((int16_t)_buffer[8]) << 8) | _buffer[9];
	GyroData[1] = (((int16_t)_buffer[10]) << 8) | _buffer[11];
	GyroData[2] = (((int16_t)_buffer[12]) << 8) | _buffer[13];
}

void MPU6500_SetClockSource(ClockSource src){
	uint8_t reg = 0;
	MPU6500_readRegister(REG_PWR_MGMT_1, &reg, 1);
	reg &= 0b11111000; 			// Clear the setting area
	reg |= (uint8_t)src;	 	// Add setting to config register value
	MPU6500_writeRegister(REG_PWR_MGMT_1, &reg, 1);
}

void MPU6500_SetSampleRateDivider(SampleRateDivider srd){
	uint8_t temp = srd;
	MPU6500_writeRegister(REG_SMPLRT_DIV,&temp,1);
}

void MPU6500_SetDLPFBandwidth(DLPFBandwidth bandwidth){
	uint8_t reg = 0;
	MPU6500_readRegister(REG_CONFIG, &reg, 1);
	reg &= 0b11111000; 			// Clear the setting area
	reg |= (uint8_t)bandwidth; 	// Add setting to config register value
	MPU6500_writeRegister(REG_CONFIG, &reg, 1);
}

void MPU6500_SetGyroRange(GyroRange range){
	uint8_t reg = 0;
	MPU6500_readRegister(REG_GYRO_CONFIG, &reg, 1);
	reg &= 0b11100111; 				// Clear the setting area
	reg |= ((uint8_t)range << 3); 	// Add setting to config register value
	MPU6500_writeRegister(REG_GYRO_CONFIG, &reg, 1);
}

void MPU6500_SetAccelRange(AccelRange range){
	uint8_t reg = 0;
	MPU6500_readRegister(REG_ACCEL_CONFIG, &reg, 1);
	reg &= 0b11100111; 				// Clear the setting area
	reg |= ((uint8_t)range << 3); 	// Add setting to config register value
	MPU6500_writeRegister(REG_ACCEL_CONFIG, &reg, 1);
}

uint8_t MPU6500_GetClockSource(void){
	uint8_t reg = 0;
	MPU6500_readRegister(REG_PWR_MGMT_1, &reg, 1);
	reg &= 0b00000111; 				// Clear unwanted area
	return reg;
}

uint8_t MPU6500_GetSampleRateDivider(void){
	uint8_t reg = 0;
	MPU6500_readRegister(REG_SMPLRT_DIV, &reg, 1);
	return reg;
}

uint8_t MPU6500_GetDLPFBandwidth(void){
	uint8_t reg = 0;
	MPU6500_readRegister(REG_CONFIG, &reg, 1);
	reg &= 0b00000111; // Clear unwanted area
	return reg;
}

uint8_t MPU6500_GetGyroRange(void){
	uint8_t reg = 0;
	MPU6500_readRegister(REG_GYRO_CONFIG, &reg, 1);
	reg &= 0b00011000;	// Clear unwanted area
	reg = reg>>3;		// Normalize
	return reg;
}

uint8_t MPU6500_GetAccelRange(void){
	uint8_t reg = 0;
	MPU6500_readRegister(REG_ACCEL_CONFIG, &reg, 1);
	reg &= 0b00011000;	// Clear unwanted area
	reg = reg>>3;		// Normalize
	return reg;
}






//float get_acc_resolution(const ACCEL_FS_SEL accel_af_sel) const {
//    switch (accel_af_sel) {
//        // Possible accelerometer scales (and their register bit settings) are:
//        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
//        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
//        case ACCEL_FS_SEL::A2G:
//            return 2.0 / 32768.0;
//        case ACCEL_FS_SEL::A4G:
//            return 4.0 / 32768.0;
//        case ACCEL_FS_SEL::A8G:
//            return 8.0 / 32768.0;
//        case ACCEL_FS_SEL::A16G:
//            return 16.0 / 32768.0;
//        default:
//            return 0.;
//    }
//}
//
//float get_gyro_resolution(const GYRO_FS_SEL gyro_fs_sel) const {
//    switch (gyro_fs_sel) {
//        // Possible gyro scales (and their register bit settings) are:
//        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
//        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
//        case GYRO_FS_SEL::G250DPS:
//            return 250.0 / 32768.0;
//        case GYRO_FS_SEL::G500DPS:
//            return 500.0 / 32768.0;
//        case GYRO_FS_SEL::G1000DPS:
//            return 1000.0 / 32768.0;
//        case GYRO_FS_SEL::G2000DPS:
//            return 2000.0 / 32768.0;
//        default:
//            return 0.;
//    }
//}
