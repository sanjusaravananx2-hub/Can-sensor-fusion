# CAN Bus Sensor Fusion Platform

**FPGA-accelerated multi-node embedded system** — an STM32 sensor node transmits IMU and environmental data over CAN bus to a Cyclone V SoC, where custom Verilog IP performs hardware-accelerated signal filtering, and the HPS ARM Cortex-A9 runs a Linux monitoring application.

## Architecture

```
┌─────────────────────┐        CAN Bus         ┌──────────────────────────────────────────┐
│   STM32F4 Black Pill│  CANH ────────────────  │          Cyclone V DE1-SoC               │
│   (Sensor Node)     │  CANL ────────────────  │                                          │
│                     │                          │  ┌──────────────────────────────────┐    │
│  ┌───────┐          │                          │  │  FPGA Fabric                     │    │
│  │MPU6050├─I2C──┐   │                          │  │                                  │    │
│  └───────┘      │   │                          │  │  SPI Master ──► CAN Frame Parser │    │
│  ┌───────┐      ▼   │   ┌────────┐             │  │  (Verilog)      (Verilog)        │    │
│  │BMP280 ├─I2C─►MCU─┼──►│MCP2515 │─CAN──────────►│           │                      │    │
│  └───────┘      │   │   │+TJA1050│             │  │      ┌────▼──────────┐           │    │
│                 │   │   └────────┘             │  │      │HW Timestamp + │           │    │
│  Bare-metal C   │   │                          │  │      │FIR Filter     │           │    │
│  Interrupt-driven   │                          │  │      └────┬──────────┘           │    │
└─────────────────────┘                          │  │           │ AXI4-Lite            │    │
                                                 │  └───────────┼──────────────────────┘    │
                                                 │              │                            │
                                                 │  ┌───────────▼──────────────────────┐    │
                                                 │  │  HPS (ARM Cortex-A9, Linux)       │    │
                                                 │  │  - Real-time data monitor          │    │
                                                 │  │  - CSV data logging                │    │
                                                 │  │  - 7-seg: CAN frame stats          │    │
                                                 │  └──────────────────────────────────┘    │
                                                 │                                          │
                                                 │  LEDs: CAN status    Switches: filter cfg│
                                                 └──────────────────────────────────────────┘
```

## Skills Demonstrated

| Skill | Implementation |
|---|---|
| Verilog RTL Design | SPI master, CAN frame parser, 8-tap FIR filter, AXI4-Lite slave |
| FPGA System Integration | Quartus Platform Designer, HPS-FPGA AXI bridges |
| CAN Bus Protocol | Real 2-node CAN 2.0B network at 500 kbps |
| Bare-metal C Firmware | STM32 interrupt-driven sensor acquisition + MCP2515 driver |
| Communication Protocols | SPI, I2C, CAN, AXI4-Lite |
| ARM Cortex-M | STM32F4 peripheral configuration, timer interrupts |
| ARM Cortex-A9 + Linux | HPS userspace app with memory-mapped FPGA I/O |
| DSP / Signal Processing | Hardware FIR filter with configurable coefficients |
| Multi-Processor Systems | STM32 (Cortex-M4) → FPGA → HPS (Cortex-A9) data pipeline |

## Bill of Materials

| Component | Purpose | Cost |
|---|---|---|
| STM32F4 Black Pill | Sensor node MCU | *Already owned* |
| Terasic DE1-SoC | FPGA + HPS processing hub | *Already owned* |
| MCP2515 + TJA1050 CAN module x2 | CAN controller + transceiver (one per node) | ~£6-8 |
| MPU6050 module | 6-axis IMU (accelerometer + gyroscope), I2C | ~£2-3 |
| BMP280 module | Temperature + barometric pressure, I2C | ~£2-3 |
| Jumper wires (M-M, M-F) | Interconnections | ~£2-3 |
| **Total** | | **~£12-17** |

Search terms for Amazon UK / eBay: "MCP2515 CAN module TJA1050", "GY-521 MPU6050 module", "BMP280 module 3.3V"

## CAN Message Protocol

| CAN ID | Data | DLC | Rate |
|---|---|---|---|
| `0x100` | Accelerometer X, Y, Z (3 x int16_t) | 6 | 100 Hz |
| `0x101` | Gyroscope X, Y, Z (3 x int16_t) | 6 | 100 Hz |
| `0x102` | Temperature (int16_t) + Pressure (uint32_t) | 6 | 10 Hz |
| `0x1FF` | Frame counter (uint16_t) + Uptime ms (uint32_t) | 6 | 1 Hz |

All frames use standard 11-bit CAN IDs, 500 kbps bitrate.

## FPGA Register Map (AXI4-Lite, base 0xFF200000)

| Offset | Name | Access | Description |
|---|---|---|---|
| 0x00 | STATUS | R | [0] frame_valid, [1] error, [15:8] error_count |
| 0x04 | CAN_ID | R | [10:0] Latest received CAN ID |
| 0x08 | CAN_DATA_LO | R | CAN data bytes [3:0] |
| 0x0C | CAN_DATA_HI | R | CAN data bytes [7:4] |
| 0x10 | TIMESTAMP | R | 32-bit hardware timestamp (1 us resolution) |
| 0x14 | ACCEL_RAW_X | R | Raw accelerometer X (signed 16-bit) |
| 0x18 | ACCEL_RAW_Y | R | Raw accelerometer Y |
| 0x1C | ACCEL_RAW_Z | R | Raw accelerometer Z |
| 0x20 | ACCEL_FILT_X | R | FIR-filtered accelerometer X (signed 32-bit) |
| 0x24 | ACCEL_FILT_Y | R | FIR-filtered accelerometer Y |
| 0x28 | ACCEL_FILT_Z | R | FIR-filtered accelerometer Z |
| 0x30 | FIR_COEFF | W | Write FIR coefficient (auto-incrementing, 8 taps) |
| 0x40 | FRAME_COUNT | R | Total CAN frames received |
| 0x44 | ERROR_COUNT | R | Total CAN errors detected |

## Project Structure

```
can-sensor-fusion-platform/
├── README.md
├── docs/
│   ├── architecture.md
│   ├── can-protocol.md
│   └── images/
├── stm32-firmware/
│   └── Core/
│       ├── Src/
│       │   ├── main.c           # Application entry, timer setup, main loop
│       │   ├── mpu6050.c        # MPU6050 I2C driver
│       │   ├── bmp280.c         # BMP280 I2C driver
│       │   ├── mcp2515.c        # MCP2515 SPI CAN controller driver
│       │   └── can_transmit.c   # CAN frame packaging and scheduling
│       └── Inc/
│           ├── mpu6050.h
│           ├── bmp280.h
│           ├── mcp2515.h
│           └── can_transmit.h
├── fpga/
│   ├── rtl/
│   │   ├── top_level.v              # Top-level: Qsys + custom logic
│   │   ├── spi_master.v             # SPI master controller
│   │   ├── mcp2515_controller.v     # MCP2515 register-level interface
│   │   ├── hw_timestamp.v           # Hardware timestamp generator
│   │   ├── fir_filter.v             # 8-tap pipelined FIR filter
│   │   ├── can_sensor_axi_slave.v   # AXI4-Lite slave register interface
│   │   └── led_status.v             # LED + 7-seg display driver
│   ├── qsys/
│   ├── constraints/
│   │   └── de1_soc_pins.tcl         # Pin assignments
│   └── quartus/
├── hps-software/
│   ├── can_monitor.c                # HPS monitoring application
│   ├── fpga_regs.h                  # FPGA register map defines
│   └── Makefile
├── testbench/
│   ├── tb_spi_master.v
│   ├── tb_fir_filter.v
│   └── tb_mcp2515_controller.v
└── hardware/
    ├── wiring.md                    # Pin-to-pin wiring guide
    └── bom.md
```

## Build Instructions

### STM32 Firmware
1. Open `stm32-firmware/` in STM32CubeIDE
2. Configure target MCU (STM32F401 or STM32F411)
3. Build and flash via ST-Link or USB DFU

### FPGA Design
1. Open `fpga/quartus/` project in Intel Quartus Prime (18.1+ recommended)
2. Generate Qsys system (Platform Designer)
3. Compile full design
4. Program DE1-SoC via JTAG (USB-Blaster)

### HPS Application
```bash
# Cross-compile for ARM (on host PC)
arm-linux-gnueabihf-gcc -o can_monitor can_monitor.c -O2

# Or compile natively on DE1-SoC
gcc -o can_monitor can_monitor.c -O2

# Run
sudo ./can_monitor
```

## License

MIT
