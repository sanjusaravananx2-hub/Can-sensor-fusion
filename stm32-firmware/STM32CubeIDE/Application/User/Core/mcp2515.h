/**
 * @file    mcp2515.h
 * @brief   MCP2515 CAN controller driver (SPI) for STM32F4
 * @details Configures MCP2515 for 500 kbps CAN 2.0B and provides TX/RX functions
 */

#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* SPI commands */
#define MCP2515_CMD_RESET       0xC0
#define MCP2515_CMD_READ        0x03
#define MCP2515_CMD_WRITE       0x02
#define MCP2515_CMD_RTS_TX0     0x81
#define MCP2515_CMD_READ_STATUS 0xA0
#define MCP2515_CMD_BIT_MODIFY  0x05
#define MCP2515_CMD_READ_RX0    0x90    /* Read RX Buffer 0 starting at SIDH */

/* Key registers */
#define MCP2515_REG_CANSTAT     0x0E
#define MCP2515_REG_CANCTRL     0x0F
#define MCP2515_REG_CNF1        0x2A
#define MCP2515_REG_CNF2        0x29
#define MCP2515_REG_CNF3        0x28
#define MCP2515_REG_CANINTE     0x2B
#define MCP2515_REG_CANINTF     0x2C
#define MCP2515_REG_TXB0CTRL    0x30
#define MCP2515_REG_TXB0SIDH    0x31
#define MCP2515_REG_TXB0SIDL    0x32
#define MCP2515_REG_TXB0DLC     0x35
#define MCP2515_REG_TXB0D0      0x36

/* Operating modes */
#define MCP2515_MODE_NORMAL     0x00
#define MCP2515_MODE_LOOPBACK   0x40
#define MCP2515_MODE_CONFIG     0x80

/* CAN frame structure */
typedef struct {
    uint16_t id;            /* Standard 11-bit CAN ID */
    uint8_t  dlc;           /* Data Length Code (0-8) */
    uint8_t  data[8];       /* Data bytes */
} CAN_Frame_t;

HAL_StatusTypeDef MCP2515_Init(void);
HAL_StatusTypeDef MCP2515_Transmit(const CAN_Frame_t *frame);
HAL_StatusTypeDef MCP2515_Receive(CAN_Frame_t *frame);
uint8_t MCP2515_FrameAvailable(void);

#endif /* MCP2515_H */
