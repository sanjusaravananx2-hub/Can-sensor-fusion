//============================================================================
// led_status.v
// LED and Seven-Segment Display Status Driver
//
// Target:  Intel Cyclone V (DE1-SoC), 50 MHz system clock
// Purpose: Provides visual feedback on DE1-SoC board:
//          - LEDR[0]   : blinks on CAN frame received (stretched pulse)
//          - LEDR[1]   : CAN error indicator (sticky)
//          - LEDR[9:2] : bargraph showing accelerometer magnitude
//          - HEX0-HEX3 : total CAN frame count in decimal (BCD)
//          - HEX4-HEX5 : frames per second (rolling 1-second window)
//
// The frame_count input comes from the register file and is converted
// to BCD for display.  A separate 1-second window counter tracks the
// instantaneous frame rate.
//============================================================================

module led_status (
    input  wire        clk,
    input  wire        rst_n,

    // Status inputs
    input  wire        can_valid,        // single-clock pulse per frame
    input  wire        can_error,        // error flag
    input  wire [31:0] frame_count,      // total frame counter
    input  wire signed [15:0] accel_x,   // raw accelerometer X

    // User switches
    input  wire [9:0]  sw,

    // LED outputs
    output reg  [9:0]  ledr,

    // Seven-segment display outputs (active low segments)
    output wire [6:0]  hex0,
    output wire [6:0]  hex1,
    output wire [6:0]  hex2,
    output wire [6:0]  hex3,
    output wire [6:0]  hex4,
    output wire [6:0]  hex5
);

    // ================================================================
    // LEDR[0] — CAN frame received indicator
    // Stretch can_valid pulse to ~50 ms for visibility
    // 50 MHz * 0.05s = 2,500,000 clocks
    // ================================================================
    localparam STRETCH_MAX = 24'd2_500_000;

    reg [23:0] stretch_cnt;
    reg        led_frame;

    always @(posedge clk) begin
        if (!rst_n) begin
            stretch_cnt <= 24'd0;
            led_frame   <= 1'b0;
        end else if (can_valid) begin
            stretch_cnt <= STRETCH_MAX;
            led_frame   <= 1'b1;
        end else if (stretch_cnt != 24'd0) begin
            stretch_cnt <= stretch_cnt - 24'd1;
        end else begin
            led_frame <= 1'b0;
        end
    end

    // ================================================================
    // LEDR[1] — CAN error indicator (sticky until reset)
    // ================================================================
    reg led_error;

    always @(posedge clk) begin
        if (!rst_n)
            led_error <= 1'b0;
        else if (can_error)
            led_error <= 1'b1;
    end

    // ================================================================
    // LEDR[9:2] — bargraph of |accel_x|
    // Use absolute value, shift right by 10 to scale ~16-bit range
    // to 8 LEDs.  Each LED represents a threshold level.
    // ================================================================
    reg [15:0] accel_abs;
    reg [7:0]  bargraph;

    always @(posedge clk) begin
        if (!rst_n) begin
            accel_abs <= 16'd0;
            bargraph  <= 8'd0;
        end else begin
            // Absolute value of accel_x
            accel_abs <= accel_x[15] ? (~accel_x + 16'd1) : accel_x;

            // Bargraph thresholds — shifted to map 16-bit range to 8 LEDs
            // Each step represents accel_abs >> 10 (divide by 1024)
            bargraph[0] <= (accel_abs > 16'd1024);
            bargraph[1] <= (accel_abs > 16'd2048);
            bargraph[2] <= (accel_abs > 16'd4096);
            bargraph[3] <= (accel_abs > 16'd6144);
            bargraph[4] <= (accel_abs > 16'd8192);
            bargraph[5] <= (accel_abs > 16'd12288);
            bargraph[6] <= (accel_abs > 16'd16384);
            bargraph[7] <= (accel_abs > 16'd24576);
        end
    end

    // Assemble LEDR
    always @(*) begin
        ledr = {bargraph, led_error, led_frame};
    end

    // ================================================================
    // Frames-per-second counter (1-second window)
    // ================================================================
    localparam ONE_SECOND = 26'd50_000_000;

    reg [25:0] fps_timer;
    reg [15:0] fps_current;    // frames counted in current window
    reg [15:0] fps_display;    // latched value for display

    always @(posedge clk) begin
        if (!rst_n) begin
            fps_timer   <= 26'd0;
            fps_current <= 16'd0;
            fps_display <= 16'd0;
        end else begin
            if (fps_timer >= ONE_SECOND - 1) begin
                fps_timer   <= 26'd0;
                fps_display <= fps_current;
                fps_current <= can_valid ? 16'd1 : 16'd0;
            end else begin
                fps_timer <= fps_timer + 26'd1;
                if (can_valid)
                    fps_current <= fps_current + 16'd1;
            end
        end
    end

    // ================================================================
    // Binary to BCD conversion — Double Dabble algorithm
    // Converts a 16-bit binary value to 5 BCD digits (20 bits)
    // ================================================================
    function [19:0] bin_to_bcd;
        input [15:0] bin;
        integer j;
        reg [35:0] scratch;  // 20 BCD bits + 16 binary bits
        begin
            scratch = 36'd0;
            scratch[15:0] = bin;
            for (j = 0; j < 16; j = j + 1) begin
                // Check each BCD digit and add 3 if >= 5
                if (scratch[19:16] >= 4'd5) scratch[19:16] = scratch[19:16] + 4'd3;
                if (scratch[23:20] >= 4'd5) scratch[23:20] = scratch[23:20] + 4'd3;
                if (scratch[27:24] >= 4'd5) scratch[27:24] = scratch[27:24] + 4'd3;
                if (scratch[31:28] >= 4'd5) scratch[31:28] = scratch[31:28] + 4'd3;
                if (scratch[35:32] >= 4'd5) scratch[35:32] = scratch[35:32] + 4'd3;
                scratch = scratch << 1;
            end
            bin_to_bcd = scratch[35:16];
        end
    endfunction

    // ================================================================
    // Seven-segment decoder
    // Input:  4-bit BCD digit (0-9)
    // Output: 7-bit segment pattern (active low for DE1-SoC)
    //         Segment mapping: {g,f,e,d,c,b,a}
    // ================================================================
    function [6:0] seven_seg_decode;
        input [3:0] digit;
        begin
            case (digit)
                4'd0:    seven_seg_decode = 7'b1000000;  // 0
                4'd1:    seven_seg_decode = 7'b1111001;  // 1
                4'd2:    seven_seg_decode = 7'b0100100;  // 2
                4'd3:    seven_seg_decode = 7'b0110000;  // 3
                4'd4:    seven_seg_decode = 7'b0011001;  // 4
                4'd5:    seven_seg_decode = 7'b0010010;  // 5
                4'd6:    seven_seg_decode = 7'b0000010;  // 6
                4'd7:    seven_seg_decode = 7'b1111000;  // 7
                4'd8:    seven_seg_decode = 7'b0000000;  // 8
                4'd9:    seven_seg_decode = 7'b0010000;  // 9
                default: seven_seg_decode = 7'b1111111;  // blank
            endcase
        end
    endfunction

    // ================================================================
    // BCD conversion and display assignment
    // ================================================================
    // Frame count BCD — lower 4 digits displayed on HEX3..HEX0
    wire [19:0] frame_bcd;
    assign frame_bcd = bin_to_bcd(frame_count[15:0]);

    // FPS BCD — 2 digits displayed on HEX5..HEX4
    wire [19:0] fps_bcd;
    assign fps_bcd = bin_to_bcd(fps_display);

    // Drive seven-segment displays
    assign hex0 = seven_seg_decode(frame_bcd[3:0]);     // ones
    assign hex1 = seven_seg_decode(frame_bcd[7:4]);     // tens
    assign hex2 = seven_seg_decode(frame_bcd[11:8]);    // hundreds
    assign hex3 = seven_seg_decode(frame_bcd[15:12]);   // thousands
    assign hex4 = seven_seg_decode(fps_bcd[3:0]);       // FPS ones
    assign hex5 = seven_seg_decode(fps_bcd[7:4]);       // FPS tens

endmodule
