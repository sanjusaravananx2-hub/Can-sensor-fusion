//============================================================================
// fir_filter.v
// 8-Tap Pipelined FIR Filter
//
// Target:  Intel Cyclone V (DE1-SoC), 50 MHz system clock
// Purpose: Low-pass filtering of accelerometer data received via CAN.
//          Coefficients are loadable at runtime from the HPS via the
//          Avalon-MM register interface.
//
// Architecture:
//   - 8-stage shift register delay line (16-bit signed samples)
//   - 8 signed 16x16 multipliers (pipelined — registered products)
//   - Adder tree to sum all products into 32-bit signed output
//   - Default coefficients: moving average (each = 512, unity = 4096)
//
// Coefficient loading:
//   coeff_wr   — write-enable pulse
//   coeff_addr — tap index [0..7]
//   coeff_data — signed 16-bit coefficient value
//
// Data interface:
//   data_in       — 16-bit signed input sample
//   data_valid    — pulse when new sample is available
//   data_out      — 32-bit signed filtered output
//   data_out_valid — pulse when output is ready (one clock after pipeline)
//============================================================================

module fir_filter (
    input  wire        clk,
    input  wire        rst_n,

    // Coefficient loading interface
    input  wire        coeff_wr,
    input  wire [2:0]  coeff_addr,
    input  wire [15:0] coeff_data,

    // Data interface
    input  wire signed [15:0] data_in,
    input  wire               data_valid,
    output reg  signed [31:0] data_out,
    output reg                data_out_valid
);

    // ----------------------------------------------------------------
    // Coefficients — default is moving average (512 per tap, unity = 4096)
    // ----------------------------------------------------------------
    reg signed [15:0] coeff [0:7];

    integer k;
    initial begin
        for (k = 0; k < 8; k = k + 1)
            coeff[k] = 16'sd512;
    end

    // Runtime coefficient loading
    always @(posedge clk) begin
        if (!rst_n) begin
            coeff[0] <= 16'sd512;
            coeff[1] <= 16'sd512;
            coeff[2] <= 16'sd512;
            coeff[3] <= 16'sd512;
            coeff[4] <= 16'sd512;
            coeff[5] <= 16'sd512;
            coeff[6] <= 16'sd512;
            coeff[7] <= 16'sd512;
        end else if (coeff_wr) begin
            coeff[coeff_addr] <= $signed(coeff_data);
        end
    end

    // ----------------------------------------------------------------
    // Delay line — 8-stage shift register
    // ----------------------------------------------------------------
    reg signed [15:0] delay [0:7];

    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 8; i = i + 1)
                delay[i] <= 16'sd0;
        end else if (data_valid) begin
            delay[0] <= data_in;
            for (i = 1; i < 8; i = i + 1)
                delay[i] <= delay[i-1];
        end
    end

    // ----------------------------------------------------------------
    // Pipeline stage 1: multiply each tap
    // 16-bit x 16-bit signed => 32-bit signed product
    // ----------------------------------------------------------------
    reg signed [31:0] prod [0:7];
    reg               pipe_valid_1;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 8; i = i + 1)
                prod[i] <= 32'sd0;
            pipe_valid_1 <= 1'b0;
        end else begin
            pipe_valid_1 <= data_valid;
            if (data_valid) begin
                for (i = 0; i < 8; i = i + 1)
                    prod[i] <= delay[i] * coeff[i];
            end
        end
    end

    // ----------------------------------------------------------------
    // Pipeline stage 2: adder tree (3 levels collapsed into one stage)
    // Sum of 8 x 32-bit products — worst case still fits 32 bits
    // for typical sensor data ranges.
    // ----------------------------------------------------------------
    reg signed [31:0] sum_01, sum_23, sum_45, sum_67;
    reg               pipe_valid_2;

    always @(posedge clk) begin
        if (!rst_n) begin
            sum_01       <= 32'sd0;
            sum_23       <= 32'sd0;
            sum_45       <= 32'sd0;
            sum_67       <= 32'sd0;
            pipe_valid_2 <= 1'b0;
        end else begin
            pipe_valid_2 <= pipe_valid_1;
            if (pipe_valid_1) begin
                sum_01 <= prod[0] + prod[1];
                sum_23 <= prod[2] + prod[3];
                sum_45 <= prod[4] + prod[5];
                sum_67 <= prod[6] + prod[7];
            end
        end
    end

    // ----------------------------------------------------------------
    // Pipeline stage 3: final sum
    // ----------------------------------------------------------------
    reg signed [31:0] sum_0123, sum_4567;
    reg               pipe_valid_3;

    always @(posedge clk) begin
        if (!rst_n) begin
            sum_0123     <= 32'sd0;
            sum_4567     <= 32'sd0;
            pipe_valid_3 <= 1'b0;
        end else begin
            pipe_valid_3 <= pipe_valid_2;
            if (pipe_valid_2) begin
                sum_0123 <= sum_01 + sum_23;
                sum_4567 <= sum_45 + sum_67;
            end
        end
    end

    // ----------------------------------------------------------------
    // Pipeline stage 4: output
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            data_out       <= 32'sd0;
            data_out_valid <= 1'b0;
        end else begin
            data_out_valid <= pipe_valid_3;
            if (pipe_valid_3) begin
                data_out <= sum_0123 + sum_4567;
            end
        end
    end

endmodule
