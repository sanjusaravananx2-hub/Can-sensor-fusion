//============================================================================
// spi_master.v
// Parameterised SPI Master Controller — Mode 0,0 (CPOL=0, CPHA=0)
//
// Target:  Intel Cyclone V (DE1-SoC), 50 MHz system clock
// Purpose: Byte-oriented SPI master used to communicate with MCP2515
//          CAN controller.  Clock rate = FCLK / (2 * CLK_DIV).
//          With CLK_DIV=8 and FCLK=50 MHz => SPI clock ~3.125 MHz.
//
// Mode 0,0:  SCLK idles LOW
//            Data sampled on RISING  edge of SCLK
//            Data shifted  on FALLING edge of SCLK
//
// Interface:
//   start   — pulse high for one clock to begin a transaction
//   tx_data — byte to transmit (latched on start)
//   rx_data — byte received (valid when done pulses)
//   done    — single-clock pulse when byte transfer completes
//   busy    — high while transfer is in progress
//   cs_n    — directly controlled: LOW while active, HIGH in idle
//
// FSM: IDLE -> ACTIVE (8-bit shift, MSB first) -> DONE -> IDLE
//============================================================================

module spi_master #(
    parameter CLK_DIV = 8          // SPI clock = FCLK / (2 * CLK_DIV)
) (
    input  wire       clk,         // system clock (50 MHz)
    input  wire       rst_n,       // active-low synchronous reset
    input  wire       start,       // pulse to begin byte transfer
    input  wire [7:0] tx_data,     // transmit data (latched on start)
    output reg  [7:0] rx_data,     // received data (valid on done)
    output reg        done,        // single-clock done pulse
    output reg        busy,        // high during transfer
    output reg        sclk,        // SPI clock output
    output reg        mosi,        // master-out-slave-in
    input  wire       miso,        // master-in-slave-out
    output reg        cs_n         // chip select (directly controlled)
);

    // ----------------------------------------------------------------
    // FSM states
    // ----------------------------------------------------------------
    localparam S_IDLE   = 2'd0;
    localparam S_ACTIVE = 2'd1;
    localparam S_DONE   = 2'd2;

    reg [1:0]  state;
    reg [7:0]  shift_tx;           // transmit shift register
    reg [7:0]  shift_rx;           // receive shift register
    reg [2:0]  bit_cnt;            // counts 0..7 bits
    reg [15:0] clk_cnt;            // clock divider counter
    reg        sclk_rising;        // pulse on rising edge of SPI clock
    reg        sclk_falling;       // pulse on falling edge of SPI clock
    reg        sclk_int;           // internal SPI clock state

    // ----------------------------------------------------------------
    // Clock divider — generates sclk_rising and sclk_falling strobes
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            clk_cnt      <= 16'd0;
            sclk_int     <= 1'b0;
            sclk_rising  <= 1'b0;
            sclk_falling <= 1'b0;
        end else if (state == S_ACTIVE) begin
            sclk_rising  <= 1'b0;
            sclk_falling <= 1'b0;
            if (clk_cnt == CLK_DIV - 1) begin
                clk_cnt  <= 16'd0;
                sclk_int <= ~sclk_int;
                if (!sclk_int)
                    sclk_rising <= 1'b1;    // about to go high
                else
                    sclk_falling <= 1'b1;   // about to go low
            end else begin
                clk_cnt <= clk_cnt + 16'd1;
            end
        end else begin
            clk_cnt      <= 16'd0;
            sclk_int     <= 1'b0;
            sclk_rising  <= 1'b0;
            sclk_falling <= 1'b0;
        end
    end

    // ----------------------------------------------------------------
    // Main FSM
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            shift_tx <= 8'd0;
            shift_rx <= 8'd0;
            bit_cnt  <= 3'd0;
            rx_data  <= 8'd0;
            done     <= 1'b0;
            busy     <= 1'b0;
            sclk     <= 1'b0;
            mosi     <= 1'b0;
            cs_n     <= 1'b1;
        end else begin
            done <= 1'b0;   // default: clear done pulse

            case (state)
                // --------------------------------------------------
                S_IDLE: begin
                    sclk <= 1'b0;          // CPOL=0: idle low
                    busy <= 1'b0;
                    if (start) begin
                        state    <= S_ACTIVE;
                        shift_tx <= tx_data;
                        shift_rx <= 8'd0;
                        bit_cnt  <= 3'd0;
                        busy     <= 1'b1;
                        cs_n     <= 1'b0;  // assert chip select
                        mosi     <= tx_data[7]; // MSB first — set up before first rising edge
                    end
                end

                // --------------------------------------------------
                S_ACTIVE: begin
                    sclk <= sclk_int;

                    // Sample MISO on rising edge (CPHA=0)
                    if (sclk_rising) begin
                        shift_rx <= {shift_rx[6:0], miso};
                    end

                    // Shift out next bit on falling edge
                    if (sclk_falling) begin
                        if (bit_cnt == 3'd7) begin
                            // All 8 bits transferred
                            state <= S_DONE;
                        end else begin
                            bit_cnt  <= bit_cnt + 3'd1;
                            shift_tx <= {shift_tx[6:0], 1'b0};
                            mosi     <= shift_tx[6]; // next bit
                        end
                    end
                end

                // --------------------------------------------------
                S_DONE: begin
                    sclk    <= 1'b0;
                    rx_data <= shift_rx;
                    done    <= 1'b1;
                    busy    <= 1'b0;
                    // NOTE: cs_n stays low — the parent module decides
                    // when to release it for multi-byte transactions.
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
