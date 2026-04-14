# Hardware Wiring Guide

## System Wiring Overview

```
                          ┌──────────────┐
    ┌──────────────┐      │   CAN Bus    │      ┌───────────────────────────────┐
    │  STM32F4     │      │   (2-wire)   │      │        DE1-SoC                │
    │  Black Pill  │      │              │      │                               │
    │              │      │  CANH ─────────────  │  GPIO_0 Header                │
    │  SPI1 ──────────── MCP2515  │      │  MCP2515 ──── GPIO_0 pins     │
    │              │      │  #1    │      │  #2              │              │
    │  I2C1 ──┐   │      │  CANL ─────────────  │              │              │
    │         │   │      │              │      │              │              │
    │     ┌───┴───┤      └──────────────┘      │  FPGA Fabric ◄──────────────┘
    │     │       │                             │              │
    │  MPU6050  BMP280                          │  HPS (Linux) │
    │  (0x68)   (0x76)                          │              │
    └──────────────┘                             └───────────────────────────────┘
```

## Node 1: STM32 Black Pill Connections

### I2C Bus (MPU6050 + BMP280 — shared bus)

| STM32 Pin | Function | MPU6050 Pin | BMP280 Pin |
|-----------|----------|-------------|------------|
| PB6       | I2C1_SCL | SCL         | SCL        |
| PB7       | I2C1_SDA | SDA         | SDA        |
| 3.3V      | Power    | VCC         | VCC        |
| GND       | Ground   | GND         | GND        |
| —         | —        | AD0 → GND   | SDO → GND  |

**Notes:**
- Both sensors share the same I2C bus (different addresses: MPU6050=0x68, BMP280=0x76)
- MPU6050 AD0 pin tied to GND sets address to 0x68
- BMP280 SDO pin tied to GND sets address to 0x76
- Use 4.7kΩ pull-up resistors on SDA and SCL to 3.3V (many breakout modules include these)

### SPI to MCP2515 Module #1

| STM32 Pin | Function   | MCP2515 Module Pin |
|-----------|------------|-------------------|
| PA5       | SPI1_SCK   | SCK               |
| PA6       | SPI1_MISO  | SO (MISO)         |
| PA7       | SPI1_MOSI  | SI (MOSI)         |
| PA4       | GPIO (CS)  | CS                |
| PA3       | GPIO (EXTI)| INT               |
| 3.3V      | Power      | VCC               |
| GND       | Ground     | GND               |

**Notes:**
- PA4 is software-controlled chip select (active low)
- PA3 configured as EXTI falling-edge interrupt input
- MCP2515 module must have 8 MHz crystal (most modules come with one)

### Status LED

| STM32 Pin | Function |
|-----------|----------|
| PC13      | Onboard LED (heartbeat, active low on Black Pill) |

## Node 2: DE1-SoC FPGA Connections

### GPIO_0 Header to MCP2515 Module #2

The DE1-SoC GPIO_0 is a 2x20 pin header. Pin numbering:

```
GPIO_0 Header (top view, looking at board edge)
┌────────────────────────────────────────┐
│  PIN1  PIN2  PIN3  PIN4  ... PIN39 PIN40│
│  (odd side)              (even side)    │
│  GPIO[0] 3.3V GPIO[2] GND  ...         │
└────────────────────────────────────────┘

Physical pin layout (2x20):
Row 1 (top):    PIN1   PIN3   PIN5   PIN7  ... PIN39
Row 2 (bottom): PIN2   PIN4   PIN6   PIN8  ... PIN40
```

| GPIO_0 Signal | FPGA Pin | Physical Header Pin | MCP2515 Module Pin | Direction |
|---------------|----------|--------------------|--------------------|-----------|
| GPIO_0[0]     | PIN_AC18 | Pin 1              | SCK                | FPGA → MCP2515 |
| GPIO_0[1]     | PIN_Y17  | Pin 3              | SI (MOSI)          | FPGA → MCP2515 |
| GPIO_0[2]     | PIN_AD17 | Pin 5              | SO (MISO)          | MCP2515 → FPGA |
| GPIO_0[3]     | PIN_Y18  | Pin 7              | CS                 | FPGA → MCP2515 |
| GPIO_0[4]     | PIN_AK16 | Pin 9              | INT                | MCP2515 → FPGA |
| 3.3V          | —        | Pin 29             | VCC                | Power |
| GND           | —        | Pin 30             | GND                | Power |

**Notes:**
- GPIO_0 pins are directly connected to FPGA I/O — operate at 3.3V LVTTL
- MCP2515 module VCC must be 3.3V (NOT 5V — the FPGA I/Os are 3.3V)
- INT pin is active-low, directly monitored by FPGA with input synchroniser

### CAN Bus Between Modules

| MCP2515 #1 (STM32 side) | MCP2515 #2 (FPGA side) | Wire Color (suggested) |
|--------------------------|------------------------|----------------------|
| CANH                     | CANH                   | Yellow               |
| CANL                     | CANL                   | Green                |
| GND                      | GND                    | Black (common ground)|

**Notes:**
- Both modules share a common ground (essential for CAN bus)
- Each MCP2515 module has an onboard TJA1050 transceiver
- Most modules have an onboard 120Ω termination resistor — check if there's a jumper
- If both modules have termination, you have the required 2x 120Ω at each end of the bus
- For short wires (<30 cm), termination is less critical but still good practice
- Keep CAN bus wires twisted together to reduce EMI

## Power Supply Notes

| Component | Voltage | Current (typical) | Power Source |
|-----------|---------|-------------------|--------------|
| STM32 Black Pill | 3.3V (onboard regulator from USB 5V) | ~50 mA | USB cable |
| MPU6050 | 3.3V | ~4 mA | STM32 3.3V pin |
| BMP280 | 3.3V | ~0.7 mA | STM32 3.3V pin |
| MCP2515 #1 | 3.3V | ~5 mA | STM32 3.3V pin |
| MCP2515 #2 | 3.3V | ~5 mA | DE1-SoC GPIO header 3.3V |
| DE1-SoC | 12V DC (onboard regulators) | ~2A | DC power adapter |

**Important:**
- Connect GND between STM32 and DE1-SoC (via CAN bus GND wire)
- Never connect 5V from STM32 USB to DE1-SoC GPIO — use only 3.3V signals
- The DE1-SoC GPIO header provides 3.3V power pins for MCP2515 #2

## Complete Wiring Checklist

- [ ] MPU6050 SDA → STM32 PB7, SCL → PB6, VCC → 3.3V, GND → GND, AD0 → GND
- [ ] BMP280 SDA → STM32 PB7, SCL → PB6, VCC → 3.3V, GND → GND, SDO → GND
- [ ] MCP2515 #1 SCK → PA5, MISO → PA6, MOSI → PA7, CS → PA4, INT → PA3, VCC → 3.3V, GND → GND
- [ ] MCP2515 #2 SCK → GPIO_0[0], MISO → GPIO_0[2], MOSI → GPIO_0[1], CS → GPIO_0[3], INT → GPIO_0[4]
- [ ] MCP2515 #2 VCC → GPIO header 3.3V (Pin 29), GND → GPIO header GND (Pin 30)
- [ ] CAN bus: CANH #1 ↔ CANH #2, CANL #1 ↔ CANL #2
- [ ] Common GND between STM32 GND and DE1-SoC GND (via CAN GND wire)
- [ ] Verify 120Ω termination on both CAN modules
