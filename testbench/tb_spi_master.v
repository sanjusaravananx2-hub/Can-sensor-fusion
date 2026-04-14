/**
 * Testbench: SPI Master Controller
 * Verifies Mode 0,0 SPI transactions with a simple loopback (MOSI -> MISO)
 */
`timescale 1ns / 1ps

module tb_spi_master;

    reg        clk;
    reg        rst_n;
    reg        start;
    reg  [7:0] tx_data;
    wire [7:0] rx_data;
    wire       done;
    wire       busy;
    wire       sclk;
    wire       mosi;
    reg        miso;
    wire       cs_n;

    /* Instantiate SPI master */
    spi_master #(
        .CLK_DIV(4)     /* Faster for simulation */
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .tx_data(tx_data),
        .rx_data(rx_data),
        .done(done),
        .busy(busy),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso),
        .cs_n(cs_n)
    );

    /* 50 MHz clock */
    initial clk = 0;
    always #10 clk = ~clk;

    /* Loopback: connect MOSI to MISO with 1 clock delay */
    always @(posedge clk) begin
        miso <= mosi;
    end

    /* Capture received data for verification */
    reg [7:0] expected_rx;

    initial begin
        $dumpfile("tb_spi_master.vcd");
        $dumpvars(0, tb_spi_master);

        /* Reset */
        rst_n = 0;
        start = 0;
        tx_data = 8'h00;
        #100;
        rst_n = 1;
        #100;

        /* Test 1: Send 0xA5 */
        $display("Test 1: Transmit 0xA5");
        tx_data = 8'hA5;
        start = 1;
        @(posedge clk);
        start = 0;

        /* Wait for completion */
        wait(done);
        @(posedge clk);
        $display("  TX: 0xA5, RX: 0x%02X (loopback delayed by 1 bit)", rx_data);

        #200;

        /* Test 2: Send 0xFF */
        $display("Test 2: Transmit 0xFF");
        tx_data = 8'hFF;
        start = 1;
        @(posedge clk);
        start = 0;

        wait(done);
        @(posedge clk);
        $display("  TX: 0xFF, RX: 0x%02X", rx_data);

        #200;

        /* Test 3: Send 0x00 */
        $display("Test 3: Transmit 0x00");
        tx_data = 8'h00;
        start = 1;
        @(posedge clk);
        start = 0;

        wait(done);
        @(posedge clk);
        $display("  TX: 0x00, RX: 0x%02X", rx_data);

        #200;

        /* Test 4: Back-to-back transfers */
        $display("Test 4: Back-to-back transfers");
        tx_data = 8'h3C;
        start = 1;
        @(posedge clk);
        start = 0;
        wait(done);
        @(posedge clk);
        $display("  Transfer 1 done, RX: 0x%02X", rx_data);

        tx_data = 8'hC3;
        start = 1;
        @(posedge clk);
        start = 0;
        wait(done);
        @(posedge clk);
        $display("  Transfer 2 done, RX: 0x%02X", rx_data);

        #500;
        $display("All SPI master tests complete.");
        $finish;
    end

endmodule
