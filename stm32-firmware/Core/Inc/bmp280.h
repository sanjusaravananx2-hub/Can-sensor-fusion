/**
 * @file    bmp280.h
 * @brief   BMP280 temperature and pressure sensor driver (I2C) for STM32F4
 * @details Reads temperature and barometric pressure at 10 Hz
 */

#ifndef BMP280_H
#define BMP280_H

#include "stm32f4xx_hal.h"

/* I2C address (SDO = GND) */
#define BMP280_ADDR             (0x76 << 1)

typedef struct {
    int32_t  temperature;   /* Temperature in 0.01 deg C (e.g., 2534 = 25.34 C) */
    uint32_t pressure;      /* Pressure in Pa/256 */
} BMP280_Data_t;

/**
 * @brief  Initialise BMP280: soft reset, read calibration, set normal mode
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef BMP280_Init(void);

/**
 * @brief  Read compensated temperature and pressure
 * @param  data  Pointer to output struct
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef BMP280_Read(BMP280_Data_t *data);

#endif /* BMP280_H */
