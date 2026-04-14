/**
 * @file    mpu6050.h
 * @brief   MPU6050 6-axis IMU driver (I2C) for STM32F4
 * @details Reads accelerometer XYZ and gyroscope XYZ at 100 Hz
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"

/* I2C address (AD0 = GND) */
#define MPU6050_ADDR            (0x68 << 1)

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp_raw;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050_Data_t;

/**
 * @brief  Initialise MPU6050: wake up, set sample rate, configure ranges
 * @retval HAL_OK on success, HAL error code on failure
 */
HAL_StatusTypeDef MPU6050_Init(void);

/**
 * @brief  Read all 6 axes (accel + gyro) + temperature in a single burst read
 * @param  data  Pointer to output struct
 * @retval HAL_OK on success, HAL error code on failure
 */
HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_Data_t *data);

#endif /* MPU6050_H */
