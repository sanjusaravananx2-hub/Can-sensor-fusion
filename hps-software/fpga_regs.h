/**
 * @file    fpga_regs.h
 * @brief   FPGA register map defines for HPS access via Lightweight H2F bridge
 * @details Base address 0xFF200000 (Lightweight HPS-to-FPGA bridge on Cyclone V)
 */

#ifndef FPGA_REGS_H
#define FPGA_REGS_H

#include <stdint.h>

/* Lightweight HPS-to-FPGA bridge base address */
#define LW_H2F_BASE        0xFF200000
#define LW_H2F_SPAN        0x00200000  /* 2 MB */

/* Register offsets (byte addresses) from component base */
#define REG_STATUS          0x00    /* [0] frame_valid, [1] error, [15:8] error_count */
#define REG_CAN_ID          0x04    /* [10:0] Latest CAN ID */
#define REG_CAN_DATA_LO     0x08    /* CAN data bytes [3:0] */
#define REG_CAN_DATA_HI     0x0C    /* CAN data bytes [7:4] */
#define REG_TIMESTAMP        0x10    /* 32-bit HW timestamp (1 us resolution) */
#define REG_ACCEL_RAW_X     0x14    /* Raw accelerometer X (signed 16-bit) */
#define REG_ACCEL_RAW_Y     0x18    /* Raw accelerometer Y */
#define REG_ACCEL_RAW_Z     0x1C    /* Raw accelerometer Z */
#define REG_ACCEL_FILT_X    0x20    /* Filtered accelerometer X (signed 32-bit) */
#define REG_ACCEL_FILT_Y    0x24    /* Filtered accelerometer Y */
#define REG_ACCEL_FILT_Z    0x28    /* Filtered accelerometer Z */
#define REG_FIR_COEFF       0x30    /* Write: {[18:16] tap_index, [15:0] coeff} */
#define REG_FRAME_COUNT     0x40    /* Total CAN frames received */
#define REG_ERROR_COUNT     0x44    /* Total CAN errors */

/* Helper macros for memory-mapped register access */
#define REG_READ(base, offset) \
    (*((volatile uint32_t *)((uint8_t *)(base) + (offset))))

#define REG_WRITE(base, offset, value) \
    (*((volatile uint32_t *)((uint8_t *)(base) + (offset))) = (value))

/* Extract signed 16-bit from 32-bit register */
#define TO_INT16(val) ((int16_t)((val) & 0xFFFF))

/* FIR coefficient packing: tap index in bits [18:16], coeff in bits [15:0] */
#define FIR_COEFF_PACK(tap, coeff) (((uint32_t)(tap) << 16) | ((coeff) & 0xFFFF))

#endif /* FPGA_REGS_H */
