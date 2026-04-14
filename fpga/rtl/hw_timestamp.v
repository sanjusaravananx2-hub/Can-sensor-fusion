//============================================================================
// hw_timestamp.v
// Hardware Microsecond Timestamp Generator
//
// Target:  Intel Cyclone V (DE1-SoC), 50 MHz system clock
// Purpose: Provides a free-running 32-bit microsecond counter.
//          On each can_valid pulse the current counter value is latched
//          to timestamp_out and timestamp_valid is pulsed for one clock.
//
// 50 MHz / 50 = 1 MHz => 1 us resolution
// 32-bit counter wraps after ~4295 seconds (~71.6 minutes)
//============================================================================

module hw_timestamp (
    input  wire        clk,              // 50 MHz system clock
    input  wire        rst_n,            // active-low synchronous reset
    input  wire        can_valid,        // single-clock pulse from CAN controller

    output reg  [31:0] timestamp_out,    // latched timestamp value
    output reg         timestamp_valid   // single-clock pulse on latch
);

    // ----------------------------------------------------------------
    // Prescaler: divide 50 MHz by 50 to get 1 MHz (1 us) tick
    // ----------------------------------------------------------------
    localparam PRESCALE = 50;

    reg [5:0]  prescale_cnt;
    reg [31:0] us_counter;

    always @(posedge clk) begin
        if (!rst_n) begin
            prescale_cnt <= 6'd0;
            us_counter   <= 32'd0;
        end else begin
            if (prescale_cnt == PRESCALE - 1) begin
                prescale_cnt <= 6'd0;
                us_counter   <= us_counter + 32'd1;
            end else begin
                prescale_cnt <= prescale_cnt + 6'd1;
            end
        end
    end

    // ----------------------------------------------------------------
    // Latch timestamp on can_valid pulse
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            timestamp_out   <= 32'd0;
            timestamp_valid <= 1'b0;
        end else begin
            timestamp_valid <= 1'b0;    // default: clear pulse

            if (can_valid) begin
                timestamp_out   <= us_counter;
                timestamp_valid <= 1'b1;
            end
        end
    end

endmodule
