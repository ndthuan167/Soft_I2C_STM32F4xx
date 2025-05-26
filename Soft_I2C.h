/**
 * @file SoftI2C.h
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Some declaration for Soft I2C (Inter-Integrated Circuit) configuration of STM32F4xx (ARMCortex M4)
 * @date 2025-05-26
 *
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"


/******************************************************************************
 * Definition
 *******************************************************************************/
#define I2C_1   0
#define I2C_2   1
#define I2C_3   2

#define I2C_MODE_WRITE  0
#define I2C_MODE_READ   1


/******************************************************************************
 * Functions Definition
 *******************************************************************************/
void SoftI2C_WriteData(uint8_t I2Cx, uint8_t Slave_address, uint16_t word_address, uint8_t *data, uint8_t size);
void SoftI2C_ReadData(uint8_t I2Cx, uint8_t Slave_address, uint8_t *data, uint8_t size);

