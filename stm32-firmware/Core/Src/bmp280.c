/**
 * @file    bmp280.c
 * @brief   BMP280 barometric pressure / temperature sensor driver (I2C, HAL).
 *
 * Reads factory-programmed calibration coefficients on init and applies
 * the Bosch datasheet compensation formulas (integer-only, 32-bit) to
 * deliver calibrated temperature (in 0.01 degC) and pressure (in Pa).
 *
 * Configuration
 * -------------
 *   - Normal mode (continuous conversion)
 *   - Temperature oversampling  x4   (osrs_t = 011)
 *   - Pressure  oversampling  x16  (osrs_p = 101)
 *   - IIR filter coefficient  16   (filter = 100)
 *   - Standby time 0.5 ms          (t_sb   = 000)
 *
 * Hardware assumptions
 * --------------------
 *   - SDO pin LOW  -> 7-bit address 0x76 (shifted to 0xEC for HAL)
 *   - Connected to I2C1 (handle declared externally as hi2c1)
 *
 * @author  Sanjeev Kumar
 * @date    2026
 */

#include "bmp280.h"

/* -------------------------------------------------------------------------
 * External peripheral handle
 * ---------------------------------------------------------------------- */
extern I2C_HandleTypeDef hi2c1;

/* -------------------------------------------------------------------------
 * BMP280 register addresses
 * ---------------------------------------------------------------------- */
#define BMP280_REG_CALIB00      0x88  /* Calibration data start (26 bytes)   */
#define BMP280_REG_CHIP_ID      0xD0  /* Chip identification (should be 0x58)*/
#define BMP280_REG_RESET        0xE0  /* Soft-reset register                 */
#define BMP280_REG_STATUS       0xF3  /* Device status                       */
#define BMP280_REG_CTRL_MEAS    0xF4  /* Oversampling + power mode           */
#define BMP280_REG_CONFIG       0xF5  /* Standby, filter, SPI3w              */
#define BMP280_REG_PRESS_MSB    0xF7  /* Pressure data start (3 bytes)       */
#define BMP280_REG_TEMP_MSB     0xFA  /* Temperature data start (3 bytes)    */

/* Expected chip ID */
#define BMP280_CHIP_ID_VAL      0x58

/* Soft-reset magic value */
#define BMP280_RESET_VAL        0xB6

/* Control register value:
 *   osrs_t[7:5] = 011  (x4 oversampling)
 *   osrs_p[4:2] = 101  (x16 oversampling)
 *   mode  [1:0] = 11   (normal mode)
 * => 0b0110_1011 = 0x6B                                                    */
#define BMP280_CTRL_MEAS_VAL    0x6B

/* Config register value:
 *   t_sb  [7:5] = 000  (0.5 ms standby)
 *   filter[4:2] = 100  (IIR coeff = 16)
 *   spi3w [0]   = 0    (disabled)
 * => 0b0001_0000 = 0x10                                                    */
#define BMP280_CONFIG_VAL       0x10

/* Calibration data length in bytes */
#define BMP280_CALIB_LEN        26

/* Raw data burst length: press(3) + temp(3) = 6 bytes from 0xF7 */
#define BMP280_RAW_DATA_LEN     6

/* I2C timeout */
#define BMP280_I2C_TIMEOUT      100

/* -------------------------------------------------------------------------
 * Module-level calibration coefficients (read once at init)
 * ---------------------------------------------------------------------- */
static uint16_t dig_T1;
static int16_t  dig_T2;
static int16_t  dig_T3;

static uint16_t dig_P1;
static int16_t  dig_P2;
static int16_t  dig_P3;
static int16_t  dig_P4;
static int16_t  dig_P5;
static int16_t  dig_P6;
static int16_t  dig_P7;
static int16_t  dig_P8;
static int16_t  dig_P9;

/**
 * @brief  Fine-temperature value shared between temperature and pressure
 *         compensation.  Updated every time BMP280_Read() is called.
 */
static int32_t t_fine;

/* -------------------------------------------------------------------------
 * Private helpers
 * ---------------------------------------------------------------------- */

/**
 * @brief  Write a single byte to a BMP280 register.
 */
static HAL_StatusTypeDef BMP280_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             BMP280_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1,
                             BMP280_I2C_TIMEOUT);
}

/**
 * @brief  Read one or more bytes starting at the given register.
 */
static HAL_StatusTypeDef BMP280_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            BMP280_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buf,
                            len,
                            BMP280_I2C_TIMEOUT);
}

/**
 * @brief  Parse factory calibration data from a 26-byte buffer.
 *
 * The calibration registers 0x88..0xA1 are stored little-endian in pairs.
 * Reference: BMP280 datasheet section 3.11.2, Table 17.
 */
static void BMP280_ParseCalibration(const uint8_t *buf)
{
    dig_T1 = (uint16_t)(buf[1]  << 8 | buf[0]);
    dig_T2 = (int16_t) (buf[3]  << 8 | buf[2]);
    dig_T3 = (int16_t) (buf[5]  << 8 | buf[4]);

    dig_P1 = (uint16_t)(buf[7]  << 8 | buf[6]);
    dig_P2 = (int16_t) (buf[9]  << 8 | buf[8]);
    dig_P3 = (int16_t) (buf[11] << 8 | buf[10]);
    dig_P4 = (int16_t) (buf[13] << 8 | buf[12]);
    dig_P5 = (int16_t) (buf[15] << 8 | buf[14]);
    dig_P6 = (int16_t) (buf[17] << 8 | buf[16]);
    dig_P7 = (int16_t) (buf[19] << 8 | buf[18]);
    dig_P8 = (int16_t) (buf[21] << 8 | buf[20]);
    dig_P9 = (int16_t) (buf[23] << 8 | buf[22]);
}

/**
 * @brief  Compensate raw temperature using 32-bit integer arithmetic.
 *
 * Taken directly from the BMP280 datasheet (section 8.1, "Compensation
 * formulas in 32-bit fixed point").  Also updates the module-level
 * t_fine variable needed for pressure compensation.
 *
 * @param  adc_T  20-bit raw temperature reading.
 * @return Temperature in units of 0.01 degC (e.g. 2347 = 23.47 degC).
 */
static int32_t BMP280_CompensateTemp(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
              ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    return T;
}

/**
 * @brief  Compensate raw pressure using 32-bit integer arithmetic.
 *
 * Must be called AFTER BMP280_CompensateTemp() so that t_fine is valid.
 *
 * @param  adc_P  20-bit raw pressure reading.
 * @return Pressure in Pa (unsigned 32-bit).  Divide by 256 for Pa with
 *         fractional part, e.g. 24674867 = 96386.2 Pa.
 */
static uint32_t BMP280_CompensatePress(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
           ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    /* Avoid division by zero */
    if (var1 == 0)
    {
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

    return (uint32_t)p;
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief  Initialise the BMP280 sensor.
 *
 * Sequence:
 *   1. Verify chip ID (0x58).
 *   2. Issue a soft reset and wait for NVM copy to complete.
 *   3. Read and parse 26 bytes of factory calibration data.
 *   4. Write CONFIG register (IIR filter, standby time) -- must be written
 *      before CTRL_MEAS transitions out of sleep mode.
 *   5. Write CTRL_MEAS register (oversampling, normal mode).
 *
 * @return HAL_OK on success.
 */
HAL_StatusTypeDef BMP280_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t chip_id = 0;
    uint8_t calib_buf[BMP280_CALIB_LEN];

    /* --- Step 1: Verify chip ID --------------------------------------- */
    status = BMP280_ReadRegs(BMP280_REG_CHIP_ID, &chip_id, 1);
    if (status != HAL_OK)
    {
        return status;
    }
    if (chip_id != BMP280_CHIP_ID_VAL)
    {
        return HAL_ERROR;
    }

    /* --- Step 2: Soft reset ------------------------------------------- */
    status = BMP280_WriteReg(BMP280_REG_RESET, BMP280_RESET_VAL);
    if (status != HAL_OK) return status;

    /* Wait for the NVM copy to finish (im_update bit in STATUS register) */
    HAL_Delay(10);
    {
        uint8_t stat = 0;
        uint8_t retries = 20;
        do
        {
            BMP280_ReadRegs(BMP280_REG_STATUS, &stat, 1);
            if ((stat & 0x01) == 0) break;  /* im_update = 0 -> done */
            HAL_Delay(2);
        } while (--retries);
    }

    /* --- Step 3: Read factory calibration ----------------------------- */
    status = BMP280_ReadRegs(BMP280_REG_CALIB00, calib_buf, BMP280_CALIB_LEN);
    if (status != HAL_OK) return status;

    BMP280_ParseCalibration(calib_buf);

    /* --- Step 4: Configure IIR filter and standby time ---------------- */
    status = BMP280_WriteReg(BMP280_REG_CONFIG, BMP280_CONFIG_VAL);
    if (status != HAL_OK) return status;

    /* --- Step 5: Set oversampling and enter normal mode ---------------- */
    status = BMP280_WriteReg(BMP280_REG_CTRL_MEAS, BMP280_CTRL_MEAS_VAL);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/**
 * @brief  Read compensated temperature and pressure from the BMP280.
 *
 * Performs a single 6-byte burst read of registers 0xF7..0xFC, then
 * applies the Bosch 32-bit integer compensation formulas.
 *
 * @param[out] data  Pointer to BMP280_Data_t struct.
 *                   - temperature_raw : compensated temp in 0.01 degC
 *                   - pressure_raw    : compensated pressure in Pa/256
 *
 * @return HAL_OK on success.
 */
HAL_StatusTypeDef BMP280_Read(BMP280_Data_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t buf[BMP280_RAW_DATA_LEN];
    int32_t adc_T, adc_P;

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    /* Burst read pressure (3 bytes) + temperature (3 bytes) ----------- */
    status = BMP280_ReadRegs(BMP280_REG_PRESS_MSB, buf, BMP280_RAW_DATA_LEN);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Assemble 20-bit raw values (MSB first, bits [19:4] in 3 bytes) -- */
    adc_P = (int32_t)(((uint32_t)buf[0] << 12) |
                      ((uint32_t)buf[1] << 4)  |
                      ((uint32_t)buf[2] >> 4));

    adc_T = (int32_t)(((uint32_t)buf[3] << 12) |
                      ((uint32_t)buf[4] << 4)  |
                      ((uint32_t)buf[5] >> 4));

    /* Compensate temperature first (updates t_fine for pressure) ------- */
    data->temperature = BMP280_CompensateTemp(adc_T);

    /* Compensate pressure (uses t_fine from temperature step) ---------- */
    data->pressure = BMP280_CompensatePress(adc_P);

    return HAL_OK;
}
