//============================================================================
// mcp2515_controller.v
// MCP2515 CAN Controller Interface — SPI-based command sequencer
//
// Target:  Intel Cyclone V (DE1-SoC), 50 MHz system clock
// Purpose: Drives an MCP2515 CAN controller through its SPI command set.
//          Initialises the device to 500 kbps (8 MHz oscillator),
//          then polls for received frames via the INT pin.
//
// MCP2515 SPI Commands used:
//   0xC0 — RESET
//   0x02 — WRITE  (followed by address, then data bytes)
//   0x03 — READ   (followed by address, returns data)
//   0x90 — READ RX BUFFER 0 (returns SIDH..D7, 13 bytes)
//   0x05 — BIT MODIFY (address, mask, data)
//
// FSM: RESET -> CONFIGURE -> IDLE -> READ_FRAME -> PARSE -> IDLE
//
// CAN frame output:
//   can_id[10:0]   — 11-bit standard identifier
//   can_data[63:0] — up to 8 data bytes (MSB = byte 0)
//   can_dlc[3:0]   — data length code
//   can_valid      — single-clock pulse when frame is ready
//   can_error      — sticky error flag (cleared on reset)
//============================================================================

module mcp2515_controller (
    input  wire        clk,          // 50 MHz system clock
    input  wire        rst_n,        // active-low synchronous reset
    input  wire        int_n,        // MCP2515 INT pin (active low)

    // SPI physical interface (directly from spi_master)
    output wire        spi_sclk,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output reg         spi_cs_n,

    // Parsed CAN frame outputs
    output reg  [10:0] can_id,
    output reg  [63:0] can_data,
    output reg  [3:0]  can_dlc,
    output reg         can_valid,
    output reg         can_error
);

    // ----------------------------------------------------------------
    // SPI master instance
    // ----------------------------------------------------------------
    reg        spi_start;
    reg  [7:0] spi_tx;
    wire [7:0] spi_rx;
    wire       spi_done;
    wire       spi_busy;
    wire       spi_cs_n_internal;   // not used — we control cs_n ourselves

    spi_master #(
        .CLK_DIV(8)
    ) u_spi (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (spi_start),
        .tx_data(spi_tx),
        .rx_data(spi_rx),
        .done   (spi_done),
        .busy   (spi_busy),
        .sclk   (spi_sclk),
        .mosi   (spi_mosi),
        .miso   (spi_miso),
        .cs_n   (spi_cs_n_internal)  // we override cs_n at top
    );

    // ----------------------------------------------------------------
    // FSM state encoding
    // ----------------------------------------------------------------
    localparam ST_RESET_INIT     = 5'd0;
    localparam ST_RESET_CMD      = 5'd1;
    localparam ST_RESET_WAIT     = 5'd2;
    localparam ST_CFG_START      = 5'd3;
    localparam ST_CFG_WRITE      = 5'd4;
    localparam ST_CFG_NEXT       = 5'd5;
    localparam ST_IDLE           = 5'd6;
    localparam ST_READ_CMD       = 5'd7;
    localparam ST_READ_BYTE      = 5'd8;
    localparam ST_READ_WAIT      = 5'd9;
    localparam ST_PARSE          = 5'd10;
    localparam ST_CLEAR_INT_CMD  = 5'd11;
    localparam ST_CLEAR_INT_WAIT = 5'd12;
    localparam ST_SPI_WAIT       = 5'd13;
    localparam ST_DEBOUNCE       = 5'd14;
    localparam ST_POLL_CMD       = 5'd15;
    localparam ST_POLL_ADDR      = 5'd16;
    localparam ST_POLL_READ      = 5'd17;
    localparam ST_POLL_CHECK     = 5'd18;

    reg [4:0]  state;
    reg [4:0]  return_state;       // state to return to after SPI byte

    // ----------------------------------------------------------------
    // Configuration ROM
    // Each entry: {address[7:0], data[7:0]}
    // MCP2515 registers for 500 kbps at 8 MHz oscillator:
    //   CNF1 (0x2A) = 0x00  (SJW=1TQ, BRP=0 => TQ = 2/Fosc = 250ns)
    //   CNF2 (0x29) = 0x90  (BTLMODE=1, SAM=0, PHSEG1=2, PRSEG=0)
    //   CNF3 (0x28) = 0x02  (PHSEG2=2)
    //   CANINTE (0x2B) = 0x01  (RX0IE — enable RX buffer 0 interrupt)
    //   RXB0CTRL (0x60) = 0x60 (RXM=11 — turn off filters, receive all)
    //   CANCTRL (0x0F) = 0x00  (normal mode, CLKOUT disabled)
    // ----------------------------------------------------------------
    localparam CFG_COUNT = 6;

    reg [15:0] cfg_rom [0:CFG_COUNT-1];
    initial begin
        cfg_rom[0] = {8'h2A, 8'h00};   // CNF1
        cfg_rom[1] = {8'h29, 8'h90};   // CNF2
        cfg_rom[2] = {8'h28, 8'h02};   // CNF3
        cfg_rom[3] = {8'h2B, 8'h01};   // CANINTE — RX0 interrupt enable
        cfg_rom[4] = {8'h60, 8'h60};   // RXB0CTRL — receive any message
        cfg_rom[5] = {8'h0F, 8'h00};   // CANCTRL — normal mode
    end

    reg [3:0]  cfg_idx;               // index into config ROM
    reg [1:0]  cfg_byte_cnt;          // byte counter within WRITE cmd

    // ----------------------------------------------------------------
    // Frame read buffer — 13 bytes from READ RX BUFFER 0
    // Byte order: SIDH, SIDL, EID8, EID0, DLC, D0..D7
    // ----------------------------------------------------------------
    reg [7:0]  rx_buf [0:12];
    reg [3:0]  rx_byte_idx;

    // ----------------------------------------------------------------
    // Delay / timeout counter
    // ----------------------------------------------------------------
    reg [23:0] delay_cnt;
    localparam RESET_DELAY = 24'd500_000;  // 10 ms at 50 MHz

    // ----------------------------------------------------------------
    // Poll timer — check MCP2515 every 50ms instead of using INT pin
    // 50 MHz * 50 ms = 2,500,000 clocks
    // ----------------------------------------------------------------
    reg [21:0] poll_timer;
    reg        poll_trigger;

    always @(posedge clk) begin
        if (!rst_n) begin
            poll_timer   <= 22'd0;
            poll_trigger <= 1'b0;
        end else if (poll_timer >= 22'd2_500_000) begin
            poll_timer   <= 22'd0;
            poll_trigger <= 1'b1;
        end else begin
            poll_timer   <= poll_timer + 22'd1;
            poll_trigger <= 1'b0;
        end
    end

    // ----------------------------------------------------------------
    // SPI byte send helper
    // To send one SPI byte: set spi_tx, pulse spi_start, wait for
    // spi_done via ST_SPI_WAIT, then jump to return_state.
    // ----------------------------------------------------------------

    // ----------------------------------------------------------------
    // Main FSM
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= ST_RESET_INIT;
            return_state <= ST_RESET_INIT;
            spi_start    <= 1'b0;
            spi_tx       <= 8'h00;
            spi_cs_n     <= 1'b1;
            can_id       <= 11'd0;
            can_data     <= 64'd0;
            can_dlc      <= 4'd0;
            can_valid    <= 1'b0;
            can_error    <= 1'b0;
            cfg_idx      <= 4'd0;
            cfg_byte_cnt <= 2'd0;
            rx_byte_idx  <= 4'd0;
            delay_cnt    <= 24'd0;
        end else begin
            spi_start <= 1'b0;      // default: no SPI start
            can_valid <= 1'b0;      // default: no frame valid pulse

            case (state)

                // ==================================================
                // RESET SEQUENCE
                // ==================================================
                ST_RESET_INIT: begin
                    // Send RESET command (0xC0)
                    spi_cs_n     <= 1'b0;
                    spi_tx       <= 8'hC0;
                    spi_start    <= 1'b1;
                    return_state <= ST_RESET_CMD;
                    state        <= ST_SPI_WAIT;
                end

                ST_RESET_CMD: begin
                    spi_cs_n  <= 1'b1;      // deassert CS after reset cmd
                    delay_cnt <= 24'd0;
                    state     <= ST_RESET_WAIT;
                end

                ST_RESET_WAIT: begin
                    // Wait for MCP2515 to complete internal reset (~10 ms)
                    if (delay_cnt >= RESET_DELAY) begin
                        state   <= ST_CFG_START;
                        cfg_idx <= 4'd0;
                    end else begin
                        delay_cnt <= delay_cnt + 24'd1;
                    end
                end

                // ==================================================
                // CONFIGURATION SEQUENCE
                // Writes each {addr, data} pair using SPI WRITE (0x02)
                // WRITE command: CS low -> 0x02 -> ADDR -> DATA -> CS high
                // ==================================================
                ST_CFG_START: begin
                    if (cfg_idx >= CFG_COUNT) begin
                        // All configuration done
                        state <= ST_IDLE;
                    end else begin
                        spi_cs_n     <= 1'b0;
                        spi_tx       <= 8'h02;       // WRITE instruction
                        spi_start    <= 1'b1;
                        cfg_byte_cnt <= 2'd0;
                        return_state <= ST_CFG_WRITE;
                        state        <= ST_SPI_WAIT;
                    end
                end

                ST_CFG_WRITE: begin
                    case (cfg_byte_cnt)
                        2'd0: begin
                            // Send address byte
                            spi_tx       <= cfg_rom[cfg_idx][15:8];
                            spi_start    <= 1'b1;
                            cfg_byte_cnt <= 2'd1;
                            return_state <= ST_CFG_WRITE;
                            state        <= ST_SPI_WAIT;
                        end
                        2'd1: begin
                            // Send data byte
                            spi_tx       <= cfg_rom[cfg_idx][7:0];
                            spi_start    <= 1'b1;
                            cfg_byte_cnt <= 2'd2;
                            return_state <= ST_CFG_NEXT;
                            state        <= ST_SPI_WAIT;
                        end
                        default: begin
                            state <= ST_CFG_NEXT;
                        end
                    endcase
                end

                ST_CFG_NEXT: begin
                    spi_cs_n <= 1'b1;          // release CS between writes
                    cfg_idx  <= cfg_idx + 4'd1;
                    state    <= ST_CFG_START;
                end

                // ==================================================
                // IDLE — wait for interrupt from MCP2515
                // ==================================================
                ST_IDLE: begin
                    spi_cs_n <= 1'b1;
                    if (poll_trigger) begin
                        // Poll: READ STATUS (0xA0) — bit 0 = RX0IF
                        spi_cs_n     <= 1'b0;
                        spi_tx       <= 8'hA0;
                        spi_start    <= 1'b1;
                        return_state <= ST_POLL_READ;
                        state        <= ST_SPI_WAIT;
                    end
                end

                // ==================================================
                // POLL — read CANINTF register to check for frame
                // READ command: CS low -> 0x03 -> 0x2C -> read byte -> CS high
                // ==================================================
                ST_POLL_CMD: begin
                    spi_cs_n     <= 1'b0;
                    spi_tx       <= 8'h03;   // READ instruction
                    spi_start    <= 1'b1;
                    cfg_byte_cnt <= 2'd0;
                    return_state <= ST_POLL_ADDR;
                    state        <= ST_SPI_WAIT;
                end

                ST_POLL_ADDR: begin
                    spi_tx       <= 8'h2C;   // CANINTF address
                    spi_start    <= 1'b1;
                    return_state <= ST_POLL_READ;
                    state        <= ST_SPI_WAIT;
                end

                ST_POLL_READ: begin
                    // Clock in status byte
                    spi_tx       <= 8'h00;
                    spi_start    <= 1'b1;
                    return_state <= ST_POLL_CHECK;
                    state        <= ST_SPI_WAIT;
                end

                ST_POLL_CHECK: begin
                    spi_cs_n <= 1'b1;
                    if (spi_rx[0]) begin
                        // Bit 0 = CANINTF.RX0IF — frame available
                        state       <= ST_READ_CMD;
                        rx_byte_idx <= 4'd0;
                    end else begin
                        state <= ST_IDLE;
                    end
                end

                // ==================================================
                // READ FRAME — Read RX Buffer 0 (0x90)
                // Command 0x90 automatically reads from RXB0SIDH
                // and returns 13 bytes: SIDH, SIDL, EID8, EID0,
                // DLC, D0, D1, D2, D3, D4, D5, D6, D7
                // CS must stay low for entire 14-byte sequence
                // (1 command + 13 data).
                // ==================================================
                ST_READ_CMD: begin
                    spi_cs_n     <= 1'b0;
                    spi_tx       <= 8'h90;   // READ RX BUFFER 0 command
                    spi_start    <= 1'b1;
                    return_state <= ST_READ_BYTE;
                    state        <= ST_SPI_WAIT;
                end

                ST_READ_BYTE: begin
                    // Send dummy byte (0x00) and receive one byte of frame
                    spi_tx       <= 8'h00;
                    spi_start    <= 1'b1;
                    return_state <= ST_READ_WAIT;
                    state        <= ST_SPI_WAIT;
                end

                ST_READ_WAIT: begin
                    // Store received byte
                    rx_buf[rx_byte_idx] <= spi_rx;
                    if (rx_byte_idx == 4'd12) begin
                        // All 13 bytes received
                        spi_cs_n <= 1'b1;
                        state    <= ST_PARSE;
                    end else begin
                        rx_byte_idx <= rx_byte_idx + 4'd1;
                        state       <= ST_READ_BYTE;
                    end
                end

                // ==================================================
                // PARSE — extract CAN ID, DLC, data from buffer
                // ==================================================
                ST_PARSE: begin
                    // Standard ID from SIDH[7:0] and SIDL[7:5]
                    // SIDH = rx_buf[0], SIDL = rx_buf[1]
                    can_id  <= {rx_buf[0], rx_buf[1][7:5]};
                    can_dlc <= rx_buf[4][3:0];
                    can_data <= {rx_buf[5],  rx_buf[6],  rx_buf[7],  rx_buf[8],
                                 rx_buf[9],  rx_buf[10], rx_buf[11], rx_buf[12]};
                    can_valid <= 1'b1;
                    state     <= ST_IDLE;  // READ RX BUFFER auto-clears RX0IF
                end

                // ==================================================
                // CLEAR RX0IF — use BIT MODIFY on CANINTF (0x2C)
                // BIT MODIFY: CS low -> 0x05 -> ADDR -> MASK -> DATA -> CS high
                // Clear bit 0 (RX0IF): mask=0x01, data=0x00
                // ==================================================
                ST_CLEAR_INT_CMD: begin
                    spi_cs_n     <= 1'b0;
                    spi_tx       <= 8'h05;   // BIT MODIFY instruction
                    spi_start    <= 1'b1;
                    cfg_byte_cnt <= 2'd0;
                    return_state <= ST_CLEAR_INT_WAIT;
                    state        <= ST_SPI_WAIT;
                end

                ST_CLEAR_INT_WAIT: begin
                    case (cfg_byte_cnt)
                        2'd0: begin
                            spi_tx       <= 8'h2C;   // CANINTF register
                            spi_start    <= 1'b1;
                            cfg_byte_cnt <= 2'd1;
                            return_state <= ST_CLEAR_INT_WAIT;
                            state        <= ST_SPI_WAIT;
                        end
                        2'd1: begin
                            spi_tx       <= 8'h01;   // mask: RX0IF
                            spi_start    <= 1'b1;
                            cfg_byte_cnt <= 2'd2;
                            return_state <= ST_CLEAR_INT_WAIT;
                            state        <= ST_SPI_WAIT;
                        end
                        2'd2: begin
                            spi_tx       <= 8'h00;   // data: clear bit
                            spi_start    <= 1'b1;
                            cfg_byte_cnt <= 2'd3;
                            return_state <= ST_CLEAR_INT_WAIT;
                            state        <= ST_SPI_WAIT;
                        end
                        2'd3: begin
                            spi_cs_n <= 1'b1;
                            state    <= ST_IDLE;
                        end
                    endcase
                end

                // ==================================================
                // DEBOUNCE — wait 5ms after clearing INT before
                // checking for next frame.  Filters noise on the
                // INT line caused by 5V MCP2515 driving 3.3V GPIO.
                // 50 MHz * 5 ms = 250,000 clocks
                // ==================================================
                ST_DEBOUNCE: begin
                    if (delay_cnt >= 24'd250_000) begin
                        state <= ST_IDLE;
                    end else begin
                        delay_cnt <= delay_cnt + 24'd1;
                    end
                end

                // ==================================================
                // SPI WAIT — waits for spi_done after issuing a byte
                // ==================================================
                ST_SPI_WAIT: begin
                    if (spi_done) begin
                        state <= return_state;
                    end
                end

                default: state <= ST_RESET_INIT;

            endcase
        end
    end

endmodule
