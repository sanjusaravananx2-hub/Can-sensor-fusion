/**
 * Testbench: 8-tap FIR Filter
 * Tests impulse response and step response to verify filter behaviour
 */
`timescale 1ns / 1ps

module tb_fir_filter;

    reg         clk;
    reg         rst_n;
    reg  [15:0] data_in;
    reg         data_valid;
    wire [31:0] data_out;
    wire        data_out_valid;
    reg         coeff_wr;
    reg  [2:0]  coeff_addr;
    reg  [15:0] coeff_data;

    /* Instantiate FIR filter */
    fir_filter uut (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(data_in),
        .data_valid(data_valid),
        .data_out(data_out),
        .data_out_valid(data_out_valid),
        .coeff_wr(coeff_wr),
        .coeff_addr(coeff_addr),
        .coeff_data(coeff_data)
    );

    /* 50 MHz clock */
    initial clk = 0;
    always #10 clk = ~clk;

    integer i;

    initial begin
        $dumpfile("tb_fir_filter.vcd");
        $dumpvars(0, tb_fir_filter);

        /* Reset */
        rst_n = 0;
        data_in = 0;
        data_valid = 0;
        coeff_wr = 0;
        coeff_addr = 0;
        coeff_data = 0;
        #100;
        rst_n = 1;
        #100;

        /* ── Test 1: Impulse Response with default coefficients (moving average) ── */
        $display("=== Test 1: Impulse Response (default moving average, coeff=512) ===");

        /* Send impulse: 4096 followed by zeros */
        data_in = 16'sd4096;   /* Impulse value */
        data_valid = 1;
        @(posedge clk);
        data_valid = 0;
        @(posedge clk);

        /* Send 10 more zeros to flush the pipeline */
        for (i = 0; i < 10; i = i + 1) begin
            data_in = 16'sd0;
            data_valid = 1;
            @(posedge clk);
            data_valid = 0;
            @(posedge clk);

            if (data_out_valid)
                $display("  Output[%0d] = %0d (expected ~2097152 for 8 taps, i.e., 4096*512)", i, $signed(data_out));
        end

        #200;

        /* ── Test 2: Step Response ── */
        $display("=== Test 2: Step Response (constant input = 1000) ===");

        for (i = 0; i < 16; i = i + 1) begin
            data_in = 16'sd1000;
            data_valid = 1;
            @(posedge clk);
            data_valid = 0;
            @(posedge clk);

            if (data_out_valid)
                $display("  Step[%0d] = %0d", i, $signed(data_out));
        end

        /* After 8 samples, output should stabilise at 1000 * 512 * 8 = 4,096,000 */
        $display("  Expected steady-state: %0d", 1000 * 512 * 8);

        #200;

        /* ── Test 3: Custom Coefficients ── */
        $display("=== Test 3: Load custom coefficients (differentiation filter) ===");

        /* Simple differentiator: coeff = {1, -1, 0, 0, 0, 0, 0, 0} scaled to {4096, -4096, ...} */
        coeff_wr = 1;
        coeff_addr = 3'd0; coeff_data = 16'sd4096; @(posedge clk);
        coeff_addr = 3'd1; coeff_data = -16'sd4096; @(posedge clk);
        coeff_addr = 3'd2; coeff_data = 16'sd0; @(posedge clk);
        coeff_addr = 3'd3; coeff_data = 16'sd0; @(posedge clk);
        coeff_addr = 3'd4; coeff_data = 16'sd0; @(posedge clk);
        coeff_addr = 3'd5; coeff_data = 16'sd0; @(posedge clk);
        coeff_addr = 3'd6; coeff_data = 16'sd0; @(posedge clk);
        coeff_addr = 3'd7; coeff_data = 16'sd0; @(posedge clk);
        coeff_wr = 0;
        @(posedge clk);

        /* Send ramp: 0, 100, 200, 300, ... */
        $display("  Ramp input with differentiator:");
        for (i = 0; i < 12; i = i + 1) begin
            data_in = i * 100;
            data_valid = 1;
            @(posedge clk);
            data_valid = 0;
            @(posedge clk);

            if (data_out_valid)
                $display("  Input=%5d, Output=%0d (expect const after ramp)", i * 100, $signed(data_out));
        end

        #200;

        /* ── Test 4: Negative values ── */
        $display("=== Test 4: Signed input handling ===");

        /* Reset to default coefficients by toggling reset */
        rst_n = 0;
        #60;
        rst_n = 1;
        #60;

        data_in = -16'sd2000;
        data_valid = 1;
        @(posedge clk);
        data_valid = 0;

        for (i = 0; i < 10; i = i + 1) begin
            data_in = 16'sd0;
            data_valid = 1;
            @(posedge clk);
            data_valid = 0;
            @(posedge clk);

            if (data_out_valid)
                $display("  Negative impulse[%0d] = %0d", i, $signed(data_out));
        end

        #500;
        $display("All FIR filter tests complete.");
        $finish;
    end

endmodule
