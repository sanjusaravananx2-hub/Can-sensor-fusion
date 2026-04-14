# ============================================================================
# Pin Assignments for DE1-SoC — CAN Sensor Fusion Platform
# ============================================================================
# Load this in Quartus: Assignments > Import Assignments
# Or source from Quartus TCL console: source de1_soc_pins.tcl
# ============================================================================

# --- Clock ---
set_location_assignment PIN_AF14 -to CLOCK_50

# --- Push Buttons (active low) ---
set_location_assignment PIN_AA14 -to KEY[0]
set_location_assignment PIN_AA15 -to KEY[1]
set_location_assignment PIN_W15  -to KEY[2]
set_location_assignment PIN_Y16  -to KEY[3]

# --- Slide Switches ---
set_location_assignment PIN_AB12 -to SW[0]
set_location_assignment PIN_AC12 -to SW[1]
set_location_assignment PIN_AF9  -to SW[2]
set_location_assignment PIN_AF10 -to SW[3]
set_location_assignment PIN_AD11 -to SW[4]
set_location_assignment PIN_AD12 -to SW[5]
set_location_assignment PIN_AE11 -to SW[6]
set_location_assignment PIN_AC9  -to SW[7]
set_location_assignment PIN_AD10 -to SW[8]
set_location_assignment PIN_AE12 -to SW[9]

# --- Red LEDs ---
set_location_assignment PIN_V16  -to LEDR[0]
set_location_assignment PIN_W16  -to LEDR[1]
set_location_assignment PIN_V17  -to LEDR[2]
set_location_assignment PIN_V18  -to LEDR[3]
set_location_assignment PIN_W17  -to LEDR[4]
set_location_assignment PIN_W19  -to LEDR[5]
set_location_assignment PIN_Y19  -to LEDR[6]
set_location_assignment PIN_W20  -to LEDR[7]
set_location_assignment PIN_W21  -to LEDR[8]
set_location_assignment PIN_Y21  -to LEDR[9]

# --- Seven Segment Displays (active low) ---
# HEX0
set_location_assignment PIN_AE26 -to HEX0[0]
set_location_assignment PIN_AE27 -to HEX0[1]
set_location_assignment PIN_AE28 -to HEX0[2]
set_location_assignment PIN_AG27 -to HEX0[3]
set_location_assignment PIN_AF28 -to HEX0[4]
set_location_assignment PIN_AG28 -to HEX0[5]
set_location_assignment PIN_AH28 -to HEX0[6]

# HEX1
set_location_assignment PIN_AJ29 -to HEX1[0]
set_location_assignment PIN_AH29 -to HEX1[1]
set_location_assignment PIN_AH30 -to HEX1[2]
set_location_assignment PIN_AG30 -to HEX1[3]
set_location_assignment PIN_AF29 -to HEX1[4]
set_location_assignment PIN_AF30 -to HEX1[5]
set_location_assignment PIN_AD27 -to HEX1[6]

# HEX2
set_location_assignment PIN_AB23 -to HEX2[0]
set_location_assignment PIN_AE29 -to HEX2[1]
set_location_assignment PIN_AD29 -to HEX2[2]
set_location_assignment PIN_AC28 -to HEX2[3]
set_location_assignment PIN_AD30 -to HEX2[4]
set_location_assignment PIN_AC29 -to HEX2[5]
set_location_assignment PIN_AC30 -to HEX2[6]

# HEX3
set_location_assignment PIN_AD26 -to HEX3[0]
set_location_assignment PIN_AC27 -to HEX3[1]
set_location_assignment PIN_AD25 -to HEX3[2]
set_location_assignment PIN_AC25 -to HEX3[3]
set_location_assignment PIN_AB28 -to HEX3[4]
set_location_assignment PIN_AB25 -to HEX3[5]
set_location_assignment PIN_AB22 -to HEX3[6]

# HEX4
set_location_assignment PIN_AA24 -to HEX4[0]
set_location_assignment PIN_Y23  -to HEX4[1]
set_location_assignment PIN_Y24  -to HEX4[2]
set_location_assignment PIN_W22  -to HEX4[3]
set_location_assignment PIN_W24  -to HEX4[4]
set_location_assignment PIN_V23  -to HEX4[5]
set_location_assignment PIN_W25  -to HEX4[6]

# HEX5
set_location_assignment PIN_V25  -to HEX5[0]
set_location_assignment PIN_AA28 -to HEX5[1]
set_location_assignment PIN_Y27  -to HEX5[2]
set_location_assignment PIN_AB27 -to HEX5[3]
set_location_assignment PIN_AB26 -to HEX5[4]
set_location_assignment PIN_AA26 -to HEX5[5]
set_location_assignment PIN_AA25 -to HEX5[6]

# --- GPIO_0 Header (directly to FPGA pins) ---
# Used for MCP2515 #2 SPI + INT connections
set_location_assignment PIN_AC18 -to GPIO_0[0]
set_location_assignment PIN_Y17  -to GPIO_0[1]
set_location_assignment PIN_AD17 -to GPIO_0[2]
set_location_assignment PIN_Y18  -to GPIO_0[3]
set_location_assignment PIN_AK16 -to GPIO_0[4]
set_location_assignment PIN_AK18 -to GPIO_0[5]
set_location_assignment PIN_AK19 -to GPIO_0[6]
set_location_assignment PIN_AJ19 -to GPIO_0[7]
set_location_assignment PIN_AJ17 -to GPIO_0[8]
set_location_assignment PIN_AJ16 -to GPIO_0[9]
set_location_assignment PIN_AH18 -to GPIO_0[10]
set_location_assignment PIN_AH17 -to GPIO_0[11]
set_location_assignment PIN_AG16 -to GPIO_0[12]
set_location_assignment PIN_AE16 -to GPIO_0[13]
set_location_assignment PIN_AF16 -to GPIO_0[14]
set_location_assignment PIN_AG17 -to GPIO_0[15]
set_location_assignment PIN_AA18 -to GPIO_0[16]
set_location_assignment PIN_AA19 -to GPIO_0[17]
set_location_assignment PIN_AE17 -to GPIO_0[18]
set_location_assignment PIN_AC20 -to GPIO_0[19]
set_location_assignment PIN_AH19 -to GPIO_0[20]
set_location_assignment PIN_AJ20 -to GPIO_0[21]
set_location_assignment PIN_AH20 -to GPIO_0[22]
set_location_assignment PIN_AK21 -to GPIO_0[23]
set_location_assignment PIN_AD19 -to GPIO_0[24]
set_location_assignment PIN_AD20 -to GPIO_0[25]
set_location_assignment PIN_AE18 -to GPIO_0[26]
set_location_assignment PIN_AE19 -to GPIO_0[27]
set_location_assignment PIN_AF20 -to GPIO_0[28]
set_location_assignment PIN_AF21 -to GPIO_0[29]
set_location_assignment PIN_AF19 -to GPIO_0[30]
set_location_assignment PIN_AG21 -to GPIO_0[31]
set_location_assignment PIN_AF18 -to GPIO_0[32]
set_location_assignment PIN_AG20 -to GPIO_0[33]
set_location_assignment PIN_AG18 -to GPIO_0[34]
set_location_assignment PIN_AJ21 -to GPIO_0[35]

# --- I/O Standard Assignments ---
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to CLOCK_50
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to KEY[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SW[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to LEDR[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX0[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX1[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX2[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX3[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX4[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX5[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to GPIO_0[*]

# --- SPI Pin-Specific Assignments ---
# GPIO_0[0] = SPI_SCK  (output to MCP2515)
# GPIO_0[1] = SPI_MOSI (output to MCP2515)
# GPIO_0[2] = SPI_MISO (input from MCP2515)
# GPIO_0[3] = SPI_CS_N (output to MCP2515)
# GPIO_0[4] = MCP2515_INT_N (input from MCP2515, active low)

# Weak pull-up on INT_N (so it stays high when MCP2515 is not asserting)
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to GPIO_0[4]
