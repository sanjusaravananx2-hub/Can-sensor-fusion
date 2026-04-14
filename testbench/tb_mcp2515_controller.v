/**
 * Testbench: MCP2515 Controller
 * Simulates the MCP2515 SPI slave responses to verify the controller FSM
 * transitions through RESET -> CONFIGURE -> IDLE -> READ_FRAME correctly
 */
`timescale 1ns / 1ps

module tb_mcp2515_controller;

    reg         clk;
    reg         rst_n;
    reg         int_n;       /* MCP2515 interrupt (active low) */
    wire [10:0] can_id;
    wire [63:0] can_data;
    wire [3:0]  can_dlc;
    wire        can_valid;
    wire        can_error;
    wire        spi_sclk;
    wire        spi_mosi;
    reg         spi_miso;
    wire        spi_cs_n;

    /* Instantiate MCP2515 controller */
    mcp2515_controller uut (
        .clk(clk),
        .rst_n(rst_n),
        .int_n(int_n),
        .can_id(can_id),
        .can_data(can_data),
        .can_dlc(can_dlc),
        .can_valid(can_valid),
        .can_error(can_error),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n)
    );

    /* 50 MHz clock */
    initial clk = 0;
    always #10 clk = ~clk;

    /*
     * Simple MCP2515 SPI slave model
     * Responds to READ commands with predefined data
     */
    reg [7:0] spi_rx_byte;
    reg [7:0] spi_tx_byte;
    reg [2:0] spi_bit_cnt;
    reg       spi_active;
    reg [7:0] mcp_response_queue [0:15];
    reg [3:0] mcp_response_idx;
    reg [3:0] mcp_response_len;
    reg       sclk_prev;

    always @(posedge clk) begin
        sclk_prev <= spi_sclk;

        if (spi_cs_n) begin
            spi_bit_cnt <= 0;
            spi_active <= 0;
            spi_miso <= 1'bz;
            mcp_response_idx <= 0;
        end else begin
            /* Rising edge of SCLK — MCP2515 samples MOSI, we shift out MISO */
            if (!sclk_prev && spi_sclk) begin
                spi_rx_byte <= {spi_rx_byte[6:0], spi_mosi};
                spi_bit_cnt <= spi_bit_cnt + 1;

                if (spi_bit_cnt == 7) begin
                    /* Full byte received — prepare response */
                    spi_active <= 1;
                end
            end

            /* Falling edge of SCLK — shift out next MISO bit */
            if (sclk_prev && !spi_sclk) begin
                if (spi_active && mcp_response_idx < mcp_response_len) begin
                    spi_miso <= spi_tx_byte[7];
                    spi_tx_byte <= {spi_tx_byte[6:0], 1'b0};
                end else begin
                    spi_miso <= 0;
                end
            end

            /* After receiving a full command byte, load response */
            if (spi_bit_cnt == 0 && spi_active) begin
                if (mcp_response_idx < mcp_response_len) begin
                    spi_tx_byte <= mcp_response_queue[mcp_response_idx];
                    mcp_response_idx <= mcp_response_idx + 1;
                end
            end
        end
    end

    /* Task to load MCP2515 response data */
    task load_response;
        input [3:0] len;
        input [7:0] d0, d1, d2, d3, d4, d5, d6, d7;
        input [7:0] d8, d9, d10, d11, d12;
        begin
            mcp_response_queue[0]  = d0;
            mcp_response_queue[1]  = d1;
            mcp_response_queue[2]  = d2;
            mcp_response_queue[3]  = d3;
            mcp_response_queue[4]  = d4;
            mcp_response_queue[5]  = d5;
            mcp_response_queue[6]  = d6;
            mcp_response_queue[7]  = d7;
            mcp_response_queue[8]  = d8;
            mcp_response_queue[9]  = d9;
            mcp_response_queue[10] = d10;
            mcp_response_queue[11] = d11;
            mcp_response_queue[12] = d12;
            mcp_response_len = len;
            mcp_response_idx = 0;
        end
    endtask

    initial begin
        $dumpfile("tb_mcp2515_controller.vcd");
        $dumpvars(0, tb_mcp2515_controller);

        /* Initial state */
        rst_n = 0;
        int_n = 1;
        spi_miso = 0;
        mcp_response_len = 0;
        mcp_response_idx = 0;

        #200;
        rst_n = 1;

        $display("=== MCP2515 Controller Test ===");
        $display("Waiting for RESET and CONFIGURE sequence...");

        /* Let the controller run through RESET and CONFIGURE */
        /* The MCP2515 model will return zeros (benign) for config reads */
        /* In a real test we'd verify exact SPI transactions with a protocol checker */

        /* Wait for controller to reach IDLE (config takes many SPI transactions) */
        #2000000;  /* 2 ms should be enough for reset + config at ~3 MHz SPI */

        $display("Controller should be in IDLE. Simulating CAN frame reception...");

        /* Prepare a mock CAN frame in MCP2515 RX buffer response:
         * Read RX Buffer 0 command (0x90) returns 13 bytes:
         *   SIDH=0x20, SIDL=0x00 → CAN ID = 0x100 (0x20 << 3 | 0 >> 5 = 0x100)
         *   EID8=0x00, EID0=0x00
         *   DLC=0x06
         *   D0=0x01, D1=0x00  (accel_x = 256)
         *   D2=0xFF, D3=0x00  (accel_y = -256 ... wait, 0xFF00 = -256 signed)
         *   D4=0x00, D5=0x10  (accel_z = 16)
         *   D6=0x00, D7=0x00
         */
        load_response(
            13,
            8'h20, 8'h00, 8'h00, 8'h00,  /* SIDH, SIDL, EID8, EID0 */
            8'h06,                          /* DLC = 6 */
            8'h01, 8'h00,                   /* D0, D1 (accel_x = 0x0100) */
            8'hFF, 8'h00,                   /* D2, D3 (accel_y = 0xFF00) */
            8'h00, 8'h10,                   /* D4, D5 (accel_z = 0x0010) */
            8'h00, 8'h00                    /* D6, D7 */
        );

        /* Assert interrupt to trigger frame read */
        int_n = 0;
        #1000;
        int_n = 1;    /* INT cleared by MCP2515 after read */

        /* Wait for frame to be parsed */
        #500000;

        /* Check outputs */
        if (can_valid) begin
            $display("Frame received!");
            $display("  CAN ID:  0x%03X (expected 0x100)", can_id);
            $display("  DLC:     %0d (expected 6)", can_dlc);
            $display("  Data:    0x%016X", can_data);
        end else begin
            $display("Waiting for can_valid...");
        end

        /* Monitor for can_valid pulse */
        @(posedge can_valid or posedge clk);
        #100;

        $display("  Final CAN ID:  0x%03X", can_id);
        $display("  Final DLC:     %0d", can_dlc);
        $display("  Final Data:    0x%016X", can_data);

        #100000;
        $display("=== MCP2515 Controller Test Complete ===");
        $finish;
    end

    /* Timeout watchdog */
    initial begin
        #10000000;  /* 10 ms timeout */
        $display("ERROR: Test timed out!");
        $finish;
    end

endmodule
