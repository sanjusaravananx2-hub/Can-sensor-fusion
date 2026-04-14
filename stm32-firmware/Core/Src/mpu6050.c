/**
 * @file    mpu6050.c
 * @brief   MPU6050 6-axis IMU driver for STM32F4 via I2C (HAL).
 *
 * Provides initialisation (wake from sleep, 100 Hz sample rate, +/-250 deg/s
 * gyro, +/-2 g accelerometer) and a single burst-read function that returns
 * accel XYZ, temperature, and gyro XYZ in one 14-byte I2C transaction.
 *
 * Hardware assumptions
 * --------------------
 *   - MPU6050 AD0 pin LOW  -> 7-bit address 0x68 (shifted to 0xD0 for HAL)
 *   - Connected to I2C1 (handle declared externally as hi2c1)
 *
 * @author  Sanjeev Kumar
 * @date    2026
 */

#include "mpu6050.h"
#include <string.h>

/* -------------------------------------------------------------------------
 * External peripheral handle
 * ---------------------------------------------------------------------- */
extern I2C_HandleTypeDef hi2c1;

/* -------------------------------------------------------------------------
 * MPU6050 register map (only the subset we need)
 * ---------------------------------------------------------------------- */
#define MPU6050_REG_SMPLRT_DIV      0x19  /* Sample Rate Divider              */
#define MPU6050_REG_CONFIG          0x1A  /* DLPF / ext sync configuration    */
#define MPU6050_REG_GYRO_CONFIG     0x1B  /* Gyroscope full-scale range       */
#define MPU6050_REG_ACCEL_CONFIG    0x1C  /* Accelerometer full-scale range   */
#define MPU6050_REG_ACCEL_XOUT_H   0x3B  /* First byte of the 14-byte burst  */
#define MPU6050_REG_PWR_MGMT_1     0x6B  /* Power management 1               */
#define MPU6050_REG_WHO_AM_I       0x75  /* Device identity (should be 0x68) */

/* Expected WHO_AM_I response */
#define MPU6050_WHO_AM_I_VAL       0x68

/* Configuration values */
#define MPU6050_WAKE_UP            0x00  /* Clear SLEEP bit in PWR_MGMT_1    */
#define MPU6050_SMPLRT_DIV_100HZ   0x09  /* Divider = 9 -> 1 kHz/(1+9)=100Hz*/
#define MPU6050_DLPF_BW_44HZ      0x03  /* DLPF config 3: ~44 Hz bandwidth  */
#define MPU6050_GYRO_FS_250        0x00  /* FS_SEL = 0 -> +/- 250 deg/s     */
#define MPU6050_ACCEL_FS_2G        0x00  /* AFS_SEL = 0 -> +/- 2 g          */

/* Burst read length: accel(6) + temp(2) + gyro(6) = 14 bytes */
#define MPU6050_BURST_LEN          14

/* I2C timeout in milliseconds */
#define MPU6050_I2C_TIMEOUT        100

/* -------------------------------------------------------------------------
 * Private helpers
 * ---------------------------------------------------------------------- */

/**
 * @brief  Write a single byte to an MPU6050 register.
 * @param  reg   Register address.
 * @param  value Byte to write.
 * @return HAL status code.
 */
static HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             MPU6050_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1,
                             MPU6050_I2C_TIMEOUT);
}

/**
 * @brief  Read a single byte from an MPU6050 register.
 * @param  reg  Register address.
 * @param  dest Pointer to store the read byte.
 * @return HAL status code.
 */
static HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg, uint8_t *dest)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            MPU6050_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            dest,
                            1,
                            MPU6050_I2C_TIMEOUT);
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief  Initialise the MPU6050.
 *
 * Sequence:
 *   1. Verify WHO_AM_I register to confirm the device is present.
 *   2. Wake the device (clear SLEEP bit in PWR_MGMT_1).
 *   3. Set sample-rate divider to 9 -> 100 Hz output rate.
 *   4. Set DLPF to ~44 Hz bandwidth (good balance for 100 Hz sampling).
 *   5. Configure gyroscope full-scale range to +/- 250 deg/s.
 *   6. Configure accelerometer full-scale range to +/- 2 g.
 *
 * @return HAL_OK on success, HAL_ERROR if the device does not respond or
 *         WHO_AM_I check fails.
 */
HAL_StatusTypeDef MPU6050_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t who_am_i = 0;

    /* --- Step 1: Verify device identity -------------------------------- */
    status = MPU6050_ReadReg(MPU6050_REG_WHO_AM_I, &who_am_i);
    if (status != HAL_OK)
    {
        return status;  /* I2C communication failure */
    }
    if (who_am_i != MPU6050_WHO_AM_I_VAL)
    {
        return HAL_ERROR;  /* Wrong device on the bus */
    }

    /* --- Step 2: Wake from sleep --------------------------------------- */
    status = MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, MPU6050_WAKE_UP);
    if (status != HAL_OK) return status;

    /* Small delay to let the oscillator stabilise after waking */
    HAL_Delay(50);

    /* --- Step 3: Sample rate divider = 9 -> 100 Hz --------------------- */
    status = MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_DIV_100HZ);
    if (status != HAL_OK) return status;

    /* --- Step 4: Digital low-pass filter ------------------------------- */
    status = MPU6050_WriteReg(MPU6050_REG_CONFIG, MPU6050_DLPF_BW_44HZ);
    if (status != HAL_OK) return status;

    /* --- Step 5: Gyroscope full-scale +/- 250 deg/s -------------------- */
    status = MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_250);
    if (status != HAL_OK) return status;

    /* --- Step 6: Accelerometer full-scale +/- 2 g ---------------------- */
    status = MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/**
 * @brief  Burst-read all sensor data from the MPU6050.
 *
 * Reads 14 contiguous bytes starting at register 0x3B:
 *   [0..1]  ACCEL_XOUT_H / L   (int16, big-endian)
 *   [2..3]  ACCEL_YOUT_H / L
 *   [4..5]  ACCEL_ZOUT_H / L
 *   [6..7]  TEMP_OUT_H   / L
 *   [8..9]  GYRO_XOUT_H  / L
 *  [10..11] GYRO_YOUT_H  / L
 *  [12..13] GYRO_ZOUT_H  / L
 *
 * Raw values are stored directly.  Conversion to physical units is left
 * to the application layer:
 *   - Accel  : raw / 16384.0  -> g        (at +/- 2 g)
 *   - Gyro   : raw / 131.0    -> deg/s    (at +/- 250 deg/s)
 *   - Temp   : raw / 340.0 + 36.53 -> degC
 *
 * @param[out] data  Pointer to MPU6050_Data_t struct to populate.
 * @return     HAL_OK on success, or the relevant HAL error code.
 */
HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_Data_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t buf[MPU6050_BURST_LEN];

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    /* Single burst read of all 14 bytes -------------------------------- */
    status = HAL_I2C_Mem_Read(&hi2c1,
                              MPU6050_ADDR,
                              MPU6050_REG_ACCEL_XOUT_H,
                              I2C_MEMADD_SIZE_8BIT,
                              buf,
                              MPU6050_BURST_LEN,
                              MPU6050_I2C_TIMEOUT);
    if (status != HAL_OK)
    {
        return status;
    }

    /* --- Parse accelerometer (big-endian, signed 16-bit) -------------- */
    data->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);

    /* --- Parse temperature -------------------------------------------- */
    data->temp_raw = (int16_t)((buf[6]  << 8) | buf[7]);

    /* --- Parse gyroscope ---------------------------------------------- */
    data->gyro_x   = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gyro_y   = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z   = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
}
