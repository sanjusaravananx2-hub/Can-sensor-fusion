# Bill of Materials

## Components to Purchase (under £20)

| # | Component | Qty | Est. Cost | Search Term (Amazon UK / eBay) |
|---|-----------|-----|-----------|-------------------------------|
| 1 | MCP2515 CAN Bus Module (with TJA1050 transceiver, 8MHz crystal) | 2 | £6-8 | "MCP2515 CAN module TJA1050" |
| 2 | MPU6050 / GY-521 6-axis IMU Module | 1 | £2-3 | "GY-521 MPU6050 module" |
| 3 | BMP280 Barometric Pressure + Temperature Module (3.3V) | 1 | £2-3 | "BMP280 module 3.3V I2C" |
| 4 | Dupont Jumper Wires (Male-Male + Male-Female, 20cm) | 1 pack | £2-3 | "dupont jumper wire kit" |
| | **Total** | | **£12-17** | |

## Already Owned

| Component | Notes |
|-----------|-------|
| STM32F4 Black Pill | Main sensor node MCU |
| Terasic DE1-SoC | Cyclone V SoC FPGA board |
| USB Micro-B cable | For STM32 power + programming |
| 12V DC adapter | For DE1-SoC |
| Breadboard | For prototyping connections |
| USB-Blaster (or built-in on DE1-SoC) | FPGA programming |

## Notes

- Ensure MCP2515 modules have **8 MHz** crystal (not 16 MHz) — the CAN timing configuration in firmware assumes 8 MHz. Most cheap modules use 8 MHz.
- Ensure BMP280 modules support **3.3V** operation. Some cheap modules are BMP280 clones (BME280 compatible). Either works.
- If MPU6050 modules come with headers unsoldered, you'll need a soldering iron.
- Check if MCP2515 modules have a **termination resistor jumper** — both should be enabled for proper CAN bus termination.
