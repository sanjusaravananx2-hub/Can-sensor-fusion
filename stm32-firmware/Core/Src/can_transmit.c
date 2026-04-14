/**
 * @file    can_transmit.c
 * @brief   CAN frame packing and transmission for the sensor fusion platform.
 *
 * Provides four transmit functions, each assembling a specific CAN frame
 * type from sensor data and forwarding it to the MCP2515 driver:
 *
 *   CAN_TX_Accel()       - 3-axis accelerometer (6 bytes, ID 0x100)
 *   CAN_TX_Gyro()        - 3-axis gyroscope     (6 bytes, ID 0x101)
 *   CAN_TX_Environment() - temperature + pressure (8 bytes, ID 0x102)
 *   CAN_TX_Heartbeat()   - frame counter + uptime (8 bytes, ID 0x1FF)
 *
 * All multi-byte fields are packed big-endian (network byte order) so
 * that any CAN bus analyser or receiving node can decode them without
 * ambiguity.
 *
 * CAN ID allocation (standard 11-bit IDs)
 * ----------------------------------------
 *   0x100  Accelerometer XYZ       int16 x 3     6 bytes
 *   0x101  Gyroscope XYZ           int16 x 3     6 bytes
 *   0x102  Environment (T + P)     int32 + uint32 8 bytes
 *   0x1FF  Heartbeat               uint32 + uint32 8 bytes
 *
 * @author  Sanjeev Kumar
 * @date    2026
 */

#include "can_transmit.h"
#include "mcp2515.h"
#include "stm32f4xx_hal.h"

/* -------------------------------------------------------------------------
 * CAN IDs -- must match the values in can_transmit.h
 * ---------------------------------------------------------------------- */
#define CAN_ID_ACCEL        0x100
#define CAN_ID_GYRO         0x101
#define CAN_ID_ENVIRONMENT  0x102
#define CAN_ID_HEARTBEAT    0x1FF

/* -------------------------------------------------------------------------
 * Private helper: pack a 16-bit signed integer big-endian
 * ---------------------------------------------------------------------- */

/**
 * @brief  Store an int16_t into a byte buffer in big-endian order.
 * @param  buf   Destination (must have room for 2 bytes).
 * @param  value The value to store.
 */
static inline void pack_int16_be(uint8_t *buf, int16_t value)
{
    buf[0] = (uint8_t)((uint16_t)value >> 8);
    buf[1] = (uint8_t)((uint16_t)value & 0xFF);
}

/**
 * @brief  Store a uint32_t into a byte buffer in big-endian order.
 * @param  buf   Destination (must have room for 4 bytes).
 * @param  value The value to store.
 */
static inline void pack_uint32_be(uint8_t *buf, uint32_t value)
{
    buf[0] = (uint8_t)(value >> 24);
    buf[1] = (uint8_t)(value >> 16);
    buf[2] = (uint8_t)(value >> 8);
    buf[3] = (uint8_t)(value & 0xFF);
}

/**
 * @brief  Store an int32_t into a byte buffer in big-endian order.
 * @param  buf   Destination (must have room for 4 bytes).
 * @param  value The value to store.
 */
static inline void pack_int32_be(uint8_t *buf, int32_t value)
{
    pack_uint32_be(buf, (uint32_t)value);
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief  Transmit an accelerometer CAN frame (ID 0x100).
 *
 * Frame layout (6 bytes):
 *   [0..1]  accel_x  int16  big-endian  (raw LSB or scaled, up to app)
 *   [2..3]  accel_y  int16  big-endian
 *   [4..5]  accel_z  int16  big-endian
 *
 * @param  ax  Accelerometer X-axis raw value.
 * @param  ay  Accelerometer Y-axis raw value.
 * @param  az  Accelerometer Z-axis raw value.
 * @return HAL status from MCP2515_Transmit().
 */
HAL_StatusTypeDef CAN_TX_Accel(int16_t ax, int16_t ay, int16_t az)
{
    CAN_Frame_t frame;

    frame.id  = CAN_ID_ACCEL;
    frame.dlc = 6;

    pack_int16_be(&frame.data[0], ax);
    pack_int16_be(&frame.data[2], ay);
    pack_int16_be(&frame.data[4], az);

    /* Zero-fill unused bytes for cleanliness */
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    return MCP2515_Transmit(&frame);
}

/**
 * @brief  Transmit a gyroscope CAN frame (ID 0x101).
 *
 * Frame layout (6 bytes):
 *   [0..1]  gyro_x  int16  big-endian
 *   [2..3]  gyro_y  int16  big-endian
 *   [4..5]  gyro_z  int16  big-endian
 *
 * @param  gx  Gyroscope X-axis raw value.
 * @param  gy  Gyroscope Y-axis raw value.
 * @param  gz  Gyroscope Z-axis raw value.
 * @return HAL status from MCP2515_Transmit().
 */
HAL_StatusTypeDef CAN_TX_Gyro(int16_t gx, int16_t gy, int16_t gz)
{
    CAN_Frame_t frame;

    frame.id  = CAN_ID_GYRO;
    frame.dlc = 6;

    pack_int16_be(&frame.data[0], gx);
    pack_int16_be(&frame.data[2], gy);
    pack_int16_be(&frame.data[4], gz);

    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    return MCP2515_Transmit(&frame);
}

/**
 * @brief  Transmit an environment (temperature + pressure) CAN frame (ID 0x102).
 *
 * Frame layout (8 bytes):
 *   [0..3]  temperature  int32   big-endian  (0.01 degC from BMP280)
 *   [4..7]  pressure     uint32  big-endian  (Pa/256 from BMP280)
 *
 * @param  temperature  Compensated temperature in 0.01 degC units.
 * @param  pressure     Compensated pressure in Pa/256 units.
 * @return HAL status from MCP2515_Transmit().
 */
HAL_StatusTypeDef CAN_TX_Environment(int32_t temperature, uint32_t pressure)
{
    CAN_Frame_t frame;

    frame.id  = CAN_ID_ENVIRONMENT;
    frame.dlc = 8;

    pack_int32_be(&frame.data[0], temperature);
    pack_uint32_be(&frame.data[4], pressure);

    return MCP2515_Transmit(&frame);
}

/**
 * @brief  Transmit a heartbeat CAN frame (ID 0x1FF).
 *
 * Contains a monotonically-increasing frame counter and the system uptime
 * in milliseconds (from HAL_GetTick()).  Useful for bus monitoring, node
 * liveness detection, and synchronisation.
 *
 * Frame layout (8 bytes):
 *   [0..3]  frame_counter  uint32  big-endian  (increments each call)
 *   [4..7]  uptime_ms      uint32  big-endian  (HAL_GetTick())
 *
 * @return HAL status from MCP2515_Transmit().
 */
HAL_StatusTypeDef CAN_TX_Heartbeat(void)
{
    static uint32_t frame_counter = 0;
    CAN_Frame_t frame;

    frame.id  = CAN_ID_HEARTBEAT;
    frame.dlc = 8;

    pack_uint32_be(&frame.data[0], frame_counter);
    pack_uint32_be(&frame.data[4], HAL_GetTick());

    frame_counter++;

    return MCP2515_Transmit(&frame);
}
