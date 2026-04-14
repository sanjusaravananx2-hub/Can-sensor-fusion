/**
 * @file    mcp2515.c
 * @brief   MCP2515 stand-alone CAN controller driver over SPI (HAL).
 *
 * Provides init, transmit, receive, and frame-available check for the
 * Microchip MCP2515 connected via SPI1.  Chip-select is active-low on
 * PA4 (GPIOA pin 4), directly managed in software.
 *
 * CAN bit-rate configuration
 * --------------------------
 *   Oscillator : 8 MHz crystal
 *   Target     : 500 kbps
 *   TQ         : 2 * (BRP+1) / Fosc  =  2 * 1 / 8 MHz = 250 ns
 *   Bit time   : SyncSeg(1) + PropSeg(1) + PS1(4) + PS2(2) = 8 TQ = 2 us
 *   Bit rate   : 1 / 2 us = 500 kbps
 *   CNF1 = 0x00  (SJW=1TQ, BRP=0)
 *   CNF2 = 0x90  (BTLMODE=1, SAM=0, PHSEG1=2, PRSEG=0)
 *   CNF3 = 0x02  (SOF=0, WAKFIL=0, PHSEG2=2)
 *
 * Hardware assumptions
 * --------------------
 *   - SPI1 (handle declared externally as hspi1)
 *   - CS   : PA4 (GPIOA, GPIO_PIN_4)
 *   - MCP2515 INT pin optionally wired to an EXTI input (not handled here)
 *
 * @author  Sanjeev Kumar
 * @date    2026
 */

#include "mcp2515.h"
#include <string.h>

/* -------------------------------------------------------------------------
 * External peripheral handle
 * ---------------------------------------------------------------------- */
extern SPI_HandleTypeDef hspi1;

/* -------------------------------------------------------------------------
 * Chip-select pin definitions
 * ---------------------------------------------------------------------- */
#define MCP2515_CS_GPIO     GPIOA
#define MCP2515_CS_PIN      GPIO_PIN_4

#define CS_LOW()   HAL_GPIO_WritePin(MCP2515_CS_GPIO, MCP2515_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(MCP2515_CS_GPIO, MCP2515_CS_PIN, GPIO_PIN_SET)

/* -------------------------------------------------------------------------
 * MCP2515 SPI instructions (from datasheet Table 12-1)
 * ---------------------------------------------------------------------- */
#define MCP_INSTR_RESET         0xC0
#define MCP_INSTR_READ          0x03
#define MCP_INSTR_WRITE         0x02
#define MCP_INSTR_BIT_MODIFY    0x05
#define MCP_INSTR_READ_STATUS   0xA0
#define MCP_INSTR_RX_STATUS     0xB0
#define MCP_INSTR_RTS_TXB0      0x81

/* -------------------------------------------------------------------------
 * MCP2515 register addresses (subset used by this driver)
 * ---------------------------------------------------------------------- */
#define MCP_REG_CANSTAT     0x0E
#define MCP_REG_CANCTRL     0x0F
#define MCP_REG_CNF3        0x28
#define MCP_REG_CNF2        0x29
#define MCP_REG_CNF1        0x2A
#define MCP_REG_CANINTE     0x2B
#define MCP_REG_CANINTF     0x2C

/* TX Buffer 0 registers */
#define MCP_REG_TXB0CTRL    0x30
#define MCP_REG_TXB0SIDH    0x31
#define MCP_REG_TXB0SIDL    0x32
#define MCP_REG_TXB0EID8    0x33
#define MCP_REG_TXB0EID0    0x34
#define MCP_REG_TXB0DLC     0x35
#define MCP_REG_TXB0D0      0x36

/* RX Buffer 0 registers */
#define MCP_REG_RXB0CTRL    0x60
#define MCP_REG_RXB0SIDH    0x61
#define MCP_REG_RXB0SIDL    0x62
#define MCP_REG_RXB0DLC     0x65
#define MCP_REG_RXB0D0      0x66

/* CANCTRL mode bits [7:5] */
#define MCP_MODE_NORMAL     0x00
#define MCP_MODE_SLEEP      0x20
#define MCP_MODE_LOOPBACK   0x40
#define MCP_MODE_LISTENONLY 0x60
#define MCP_MODE_CONFIG     0x80
#define MCP_MODE_MASK       0xE0

/* CANINTE / CANINTF bits */
#define MCP_INT_RX0IF       0x01
#define MCP_INT_RX1IF       0x02
#define MCP_INT_TX0IF       0x04

/* TXBnCTRL bits */
#define MCP_TXREQ           0x08

/* Bit-rate configuration for 500 kbps @ 8 MHz */
#define MCP_CNF1_500K       0x00
#define MCP_CNF2_500K       0x90
#define MCP_CNF3_500K       0x02

/* SPI timeout */
#define MCP_SPI_TIMEOUT     100

/* Max retries waiting for mode change */
#define MCP_MODE_RETRIES    10

/* -------------------------------------------------------------------------
 * Private SPI helpers
 * ---------------------------------------------------------------------- */

/**
 * @brief  Read a single MCP2515 register.
 * @param  addr  Register address.
 * @return Register value.
 */
static uint8_t MCP2515_ReadRegister(uint8_t addr)
{
    uint8_t tx[3] = { MCP_INSTR_READ, addr, 0x00 };
    uint8_t rx[3] = { 0 };

    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, MCP_SPI_TIMEOUT);
    CS_HIGH();

    return rx[2];  /* Data byte is clocked out on the third byte */
}

/**
 * @brief  Write a single MCP2515 register.
 * @param  addr  Register address.
 * @param  value Byte to write.
 */
static void MCP2515_WriteRegister(uint8_t addr, uint8_t value)
{
    uint8_t tx[3] = { MCP_INSTR_WRITE, addr, value };

    CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 3, MCP_SPI_TIMEOUT);
    CS_HIGH();
}

/**
 * @brief  Write multiple contiguous MCP2515 registers.
 * @param  addr   Starting register address.
 * @param  data   Pointer to data bytes.
 * @param  len    Number of bytes to write.
 */
static void MCP2515_WriteRegisters(uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t hdr[2] = { MCP_INSTR_WRITE, addr };

    CS_LOW();
    HAL_SPI_Transmit(&hspi1, hdr, 2, MCP_SPI_TIMEOUT);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, MCP_SPI_TIMEOUT);
    CS_HIGH();
}

/**
 * @brief  Read multiple contiguous MCP2515 registers.
 * @param  addr  Starting register address.
 * @param  buf   Destination buffer.
 * @param  len   Number of bytes to read.
 */
static void MCP2515_ReadRegisters(uint8_t addr, uint8_t *buf, uint8_t len)
{
    uint8_t hdr[2] = { MCP_INSTR_READ, addr };

    CS_LOW();
    HAL_SPI_Transmit(&hspi1, hdr, 2, MCP_SPI_TIMEOUT);
    /* Clock out dummy bytes while reading */
    memset(buf, 0x00, len);
    HAL_SPI_Receive(&hspi1, buf, len, MCP_SPI_TIMEOUT);
    CS_HIGH();
}

/**
 * @brief  Modify specific bits within an MCP2515 register.
 * @param  addr  Register address (must support bit modify -- see datasheet).
 * @param  mask  Bit mask (1 = modify, 0 = keep).
 * @param  value New bit values (only bits set in mask are applied).
 */
static void MCP2515_BitModify(uint8_t addr, uint8_t mask, uint8_t value)
{
    uint8_t tx[4] = { MCP_INSTR_BIT_MODIFY, addr, mask, value };

    CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 4, MCP_SPI_TIMEOUT);
    CS_HIGH();
}

/**
 * @brief  Issue the SPI RESET instruction to the MCP2515.
 *
 * After reset the device enters Configuration Mode automatically.
 */
static void MCP2515_Reset(void)
{
    uint8_t cmd = MCP_INSTR_RESET;

    CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, MCP_SPI_TIMEOUT);
    CS_HIGH();

    HAL_Delay(10);  /* Allow oscillator start-up */
}

/**
 * @brief  Set the MCP2515 operating mode and wait until it takes effect.
 * @param  mode  One of MCP_MODE_NORMAL, MCP_MODE_CONFIG, etc.
 * @return HAL_OK if mode was entered, HAL_TIMEOUT otherwise.
 */
static HAL_StatusTypeDef MCP2515_SetMode(uint8_t mode)
{
    MCP2515_BitModify(MCP_REG_CANCTRL, MCP_MODE_MASK, mode);

    /* Poll CANSTAT until the mode bits match */
    for (uint8_t i = 0; i < MCP_MODE_RETRIES; i++)
    {
        uint8_t canstat = MCP2515_ReadRegister(MCP_REG_CANSTAT);
        if ((canstat & MCP_MODE_MASK) == mode)
        {
            return HAL_OK;
        }
        HAL_Delay(1);
    }

    return HAL_TIMEOUT;
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief  Initialise the MCP2515 CAN controller.
 *
 * Sequence:
 *   1. Hardware reset via SPI command (enters config mode).
 *   2. Set bit-rate registers for 500 kbps @ 8 MHz oscillator.
 *   3. Clear all TX/RX interrupt flags.
 *   4. Enable RX0 interrupt (optional, for future polled/IRQ use).
 *   5. Set RXB0CTRL to accept all messages (no filter/mask).
 *   6. Switch to Normal Mode.
 *
 * @return HAL_OK on success, HAL_TIMEOUT if mode switch fails.
 */
HAL_StatusTypeDef MCP2515_Init(void)
{
    HAL_StatusTypeDef status;

    /* Ensure CS starts high (deselected) */
    CS_HIGH();

    /* --- Step 1: Reset (enters Configuration Mode) -------------------- */
    MCP2515_Reset();

    /* Verify we are in Configuration Mode */
    status = MCP2515_SetMode(MCP_MODE_CONFIG);
    if (status != HAL_OK) return status;

    /* --- Step 2: Bit-rate configuration ------------------------------- */
    MCP2515_WriteRegister(MCP_REG_CNF1, MCP_CNF1_500K);
    MCP2515_WriteRegister(MCP_REG_CNF2, MCP_CNF2_500K);
    MCP2515_WriteRegister(MCP_REG_CNF3, MCP_CNF3_500K);

    /* --- Step 3: Clear all interrupt flags ----------------------------- */
    MCP2515_WriteRegister(MCP_REG_CANINTF, 0x00);

    /* --- Step 4: Enable RX0 receive interrupt ------------------------- */
    MCP2515_WriteRegister(MCP_REG_CANINTE, MCP_INT_RX0IF);

    /* --- Step 5: RXB0 accept any message (RXM[1:0] = 11) ------------- */
    MCP2515_WriteRegister(MCP_REG_RXB0CTRL, 0x60);

    /* --- Step 6: Enter Normal Mode ------------------------------------ */
    status = MCP2515_SetMode(MCP_MODE_NORMAL);
    return status;
}

/**
 * @brief  Transmit a standard CAN frame via TX Buffer 0.
 *
 * Loads the 11-bit standard identifier, DLC, and up to 8 data bytes into
 * TXB0, then issues a Request-to-Send via the single-byte SPI instruction.
 * Polls TXREQ until the frame is sent (or a short timeout expires).
 *
 * @param[in] frame  Pointer to a CAN_Frame_t to transmit.
 * @return    HAL_OK on success, HAL_TIMEOUT if TX did not complete.
 */
HAL_StatusTypeDef MCP2515_Transmit(const CAN_Frame_t *frame)
{
    uint8_t txbuf[13];  /* SIDH + SIDL + EID8 + EID0 + DLC + D0..D7 */
    uint8_t dlc;
    uint8_t cmd;

    if (frame == NULL) return HAL_ERROR;

    dlc = frame->dlc;
    if (dlc > 8) dlc = 8;

    /* --- Build the register image for TXB0 (SIDH..D7) ----------------- */

    /* Standard ID: SIDH holds bits [10:3], SIDL holds bits [2:0] << 5 */
    txbuf[0] = (uint8_t)(frame->id >> 3);           /* TXB0SIDH */
    txbuf[1] = (uint8_t)((frame->id & 0x07) << 5);  /* TXB0SIDL (EXIDE=0) */
    txbuf[2] = 0x00;                                  /* TXB0EID8 (unused)  */
    txbuf[3] = 0x00;                                  /* TXB0EID0 (unused)  */
    txbuf[4] = dlc;                                    /* TXB0DLC           */

    /* Copy payload */
    memcpy(&txbuf[5], frame->data, dlc);

    /* Write all registers in one burst (SIDH is at TXB0SIDH = 0x31) */
    MCP2515_WriteRegisters(MCP_REG_TXB0SIDH, txbuf, 5 + dlc);

    /* --- Issue Request-to-Send for TXB0 ------------------------------- */
    cmd = MCP_INSTR_RTS_TXB0;
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, MCP_SPI_TIMEOUT);
    CS_HIGH();

    /* --- Wait for transmission to complete ----------------------------- */
    for (uint8_t i = 0; i < 50; i++)
    {
        uint8_t ctrl = MCP2515_ReadRegister(MCP_REG_TXB0CTRL);
        if ((ctrl & MCP_TXREQ) == 0)
        {
            /* Clear the TX interrupt flag */
            MCP2515_BitModify(MCP_REG_CANINTF, MCP_INT_TX0IF, 0x00);
            return HAL_OK;
        }
        /* Brief spin -- each iteration is ~50 us at 100 MHz core */
    }

    return HAL_TIMEOUT;
}

/**
 * @brief  Receive a CAN frame from RX Buffer 0.
 *
 * Should be called after MCP2515_FrameAvailable() returns non-zero.
 * Reads the standard ID, DLC, and data bytes from RXB0, then clears
 * the RX0IF interrupt flag so new frames can be received.
 *
 * @param[out] frame  Pointer to CAN_Frame_t to populate.
 * @return     HAL_OK on success, HAL_ERROR if no frame is available.
 */
HAL_StatusTypeDef MCP2515_Receive(CAN_Frame_t *frame)
{
    uint8_t rxbuf[13];  /* SIDH + SIDL + EID8 + EID0 + DLC + D0..D7 */
    uint8_t dlc;

    if (frame == NULL) return HAL_ERROR;

    /* Check if there is actually a frame waiting */
    uint8_t intf = MCP2515_ReadRegister(MCP_REG_CANINTF);
    if ((intf & MCP_INT_RX0IF) == 0)
    {
        return HAL_ERROR;  /* No frame in RXB0 */
    }

    /* Burst read RXB0: SIDH(0x61)..D7(0x6D) = 13 bytes */
    MCP2515_ReadRegisters(MCP_REG_RXB0SIDH, rxbuf, 13);

    /* Parse standard ID from SIDH and SIDL */
    frame->id = ((uint16_t)rxbuf[0] << 3) |
                ((uint16_t)(rxbuf[1] >> 5) & 0x07);

    /* DLC (lower nibble only) */
    dlc = rxbuf[4] & 0x0F;
    if (dlc > 8) dlc = 8;
    frame->dlc = dlc;

    /* Copy payload */
    memcpy(frame->data, &rxbuf[5], dlc);

    /* Clear the RX0 interrupt flag to allow the next reception */
    MCP2515_BitModify(MCP_REG_CANINTF, MCP_INT_RX0IF, 0x00);

    return HAL_OK;
}

/**
 * @brief  Check whether a received CAN frame is waiting in RX Buffer 0.
 *
 * This is a lightweight polling function. It reads the CANINTF register
 * and checks the RX0IF bit.
 *
 * @return 1 if a frame is available, 0 otherwise.
 */
uint8_t MCP2515_FrameAvailable(void)
{
    uint8_t intf = MCP2515_ReadRegister(MCP_REG_CANINTF);
    return (intf & MCP_INT_RX0IF) ? 1 : 0;
}
