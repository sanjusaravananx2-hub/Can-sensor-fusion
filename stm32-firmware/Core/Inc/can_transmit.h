/**
 * @file    can_transmit.h
 * @brief   CAN frame packaging and transmission scheduler
 * @details Packages sensor data into CAN frames and transmits at defined rates
 */

#ifndef CAN_TRANSMIT_H
#define CAN_TRANSMIT_H

#include <stdint.h>
#include "mpu6050.h"
#include "bmp280.h"

/* CAN message IDs */
#define CAN_ID_ACCEL        0x100   /* MPU6050 accelerometer XYZ, 100 Hz */
#define CAN_ID_GYRO         0x101   /* MPU6050 gyroscope XYZ, 100 Hz */
#define CAN_ID_ENV          0x102   /* BMP280 temperature + pressure, 10 Hz */
#define CAN_ID_HEARTBEAT    0x1FF   /* Frame counter + uptime, 1 Hz */

/**
 * @brief  Package and transmit accelerometer data as CAN frame (ID 0x100)
 * @param  data  Pointer to MPU6050 data (accel X, Y, Z used)
 */
void CAN_TX_Accel(const MPU6050_Data_t *data);

/**
 * @brief  Package and transmit gyroscope data as CAN frame (ID 0x101)
 * @param  data  Pointer to MPU6050 data (gyro X, Y, Z used)
 */
void CAN_TX_Gyro(const MPU6050_Data_t *data);

/**
 * @brief  Package and transmit environment data as CAN frame (ID 0x102)
 * @param  data  Pointer to BMP280 data
 */
void CAN_TX_Environment(const BMP280_Data_t *data);

/**
 * @brief  Transmit heartbeat frame with frame counter and uptime (ID 0x1FF)
 */
void CAN_TX_Heartbeat(void);

#endif /* CAN_TRANSMIT_H */
