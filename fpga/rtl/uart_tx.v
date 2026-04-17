//============================================================================
// uart_tx.v
// Simple UART Transmitter — 115200 baud, 8N1
//
// Target:  50 MHz system clock
// Baud:    115200 → 50_000_000 / 115200 ≈ 434 clocks per bit
//============================================================================

module uart_tx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    input  wire       valid,      // pulse high for 1 clock to send
    output reg        tx,
    output wire       busy
);

    localparam CLKS_PER_BIT = 434;  // 50MHz / 115200

    localparam ST_IDLE  = 2'd0;
    localparam ST_START = 2'd1;
    localparam ST_DATA  = 2'd2;
    localparam ST_STOP  = 2'd3;

    reg [1:0]  state;
    reg [8:0]  clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shift_reg;

    assign busy = (state != ST_IDLE);

    always @(posedge clk) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            tx        <= 1'b1;      // idle high
            clk_cnt   <= 9'd0;
            bit_idx   <= 3'd0;
            shift_reg <= 8'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    tx <= 1'b1;
                    if (valid) begin
                        shift_reg <= data;
                        state     <= ST_START;
                        clk_cnt   <= 9'd0;
                    end
                end

                ST_START: begin
                    tx <= 1'b0;     // start bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 9'd0;
                        bit_idx <= 3'd0;
                        state   <= ST_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 9'd1;
                    end
                end

                ST_DATA: begin
                    tx <= shift_reg[bit_idx];   // LSB first
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 9'd0;
                        if (bit_idx == 3'd7)
                            state <= ST_STOP;
                        else
                            bit_idx <= bit_idx + 3'd1;
                    end else begin
                        clk_cnt <= clk_cnt + 9'd1;
                    end
                end

                ST_STOP: begin
                    tx <= 1'b1;     // stop bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        state <= ST_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 9'd1;
                    end
                end
            endcase
        end
    end

endmodule
