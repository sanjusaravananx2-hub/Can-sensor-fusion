//============================================================================
// can_uart_monitor.v
// Sends CAN frame data over UART at 115200 baud
//
// Output format (one line per frame):
//   F:NNNN ID:XXX AX:SXXXX AY:SXXXX AZ:SXXXX\r\n
//
// Where NNNN = frame count, XXX = CAN ID (hex), SXXXX = signed accel
//============================================================================

module can_uart_monitor (
    input  wire        clk,
    input  wire        rst_n,

    // CAN data inputs
    input  wire        can_valid,
    input  wire [10:0] can_id,
    input  wire [63:0] can_data,
    input  wire [31:0] frame_count,
    input  wire signed [15:0] accel_x,
    input  wire signed [15:0] accel_y,
    input  wire signed [15:0] accel_z,

    // UART output
    output wire        uart_tx
);

    // UART transmitter
    reg  [7:0] tx_data;
    reg        tx_valid;
    wire       tx_busy;

    uart_tx u_uart (
        .clk   (clk),
        .rst_n (rst_n),
        .data  (tx_data),
        .valid (tx_valid),
        .tx    (uart_tx),
        .busy  (tx_busy)
    );

    // ================================================================
    // Message buffer — build the string, send byte by byte
    // ================================================================
    // Max message: "F:9999 ID:100 AX:-32768 AY:-32768 AZ:-32768\r\n" = ~50 chars
    reg [7:0] msg_buf [0:63];
    reg [5:0] msg_len;
    reg [5:0] msg_idx;

    // FSM
    localparam ST_IDLE    = 2'd0;
    localparam ST_BUILD   = 2'd1;
    localparam ST_SEND    = 2'd2;
    localparam ST_WAIT    = 2'd3;

    reg [1:0] state;

    // Latch CAN data on can_valid
    reg        frame_pending;
    reg [31:0] lat_fcount;
    reg [10:0] lat_id;
    reg signed [15:0] lat_ax, lat_ay, lat_az;

    always @(posedge clk) begin
        if (!rst_n)
            frame_pending <= 1'b0;
        else if (can_valid && state == ST_IDLE) begin
            frame_pending <= 1'b1;
            lat_fcount    <= frame_count;
            lat_id        <= can_id;
            lat_ax        <= accel_x;
            lat_ay        <= accel_y;
            lat_az        <= accel_z;
        end else if (state == ST_BUILD)
            frame_pending <= 1'b0;
    end

    // ================================================================
    // Helper: convert nibble to hex ASCII
    // ================================================================
    function [7:0] hex_char;
        input [3:0] nibble;
        begin
            if (nibble < 4'd10)
                hex_char = 8'd48 + {4'd0, nibble};  // '0'-'9'
            else
                hex_char = 8'd55 + {4'd0, nibble};  // 'A'-'F'
        end
    endfunction

    // ================================================================
    // Helper task-like logic: store decimal signed 16-bit at position
    // We use a simple approach: convert abs value, prepend sign
    // ================================================================
    reg [15:0] abs_val;
    reg [5:0]  build_pos;

    // Decimal digits extraction (5 digits max for 16-bit: 0-65535)
    wire [3:0] dig4 = abs_val / 10000;
    wire [3:0] dig3 = (abs_val / 1000) % 10;
    wire [3:0] dig2 = (abs_val / 100) % 10;
    wire [3:0] dig1 = (abs_val / 10) % 10;
    wire [3:0] dig0 = abs_val % 10;

    // Frame count digits
    wire [3:0] fc3 = lat_fcount / 1000;
    wire [3:0] fc2 = (lat_fcount / 100) % 10;
    wire [3:0] fc1 = (lat_fcount / 10) % 10;
    wire [3:0] fc0 = lat_fcount % 10;

    // Build phase counter
    reg [3:0] build_step;

    always @(posedge clk) begin
        if (!rst_n) begin
            state    <= ST_IDLE;
            tx_valid <= 1'b0;
            msg_idx  <= 6'd0;
            msg_len  <= 6'd0;
            build_step <= 4'd0;
        end else begin
            tx_valid <= 1'b0;  // default

            case (state)
                ST_IDLE: begin
                    if (frame_pending) begin
                        state      <= ST_BUILD;
                        build_pos  <= 6'd0;
                        build_step <= 4'd0;
                    end
                end

                ST_BUILD: begin
                    // Build message in msg_buf one step at a time
                    case (build_step)
                        4'd0: begin
                            // "F:"
                            msg_buf[0] <= "F";
                            msg_buf[1] <= ":";
                            // Frame count 4 digits
                            msg_buf[2] <= 8'd48 + {4'd0, fc3};
                            msg_buf[3] <= 8'd48 + {4'd0, fc2};
                            msg_buf[4] <= 8'd48 + {4'd0, fc1};
                            msg_buf[5] <= 8'd48 + {4'd0, fc0};
                            msg_buf[6] <= " ";
                            // "ID:"
                            msg_buf[7]  <= "I";
                            msg_buf[8]  <= "D";
                            msg_buf[9]  <= ":";
                            msg_buf[10] <= hex_char(lat_id[10:8]);
                            msg_buf[11] <= hex_char(lat_id[7:4]);
                            msg_buf[12] <= hex_char(lat_id[3:0]);
                            msg_buf[13] <= " ";
                            build_step <= 4'd1;
                            abs_val <= lat_ax[15] ? (~lat_ax + 16'd1) : lat_ax;
                        end

                        4'd1: begin
                            // "AX:" + sign + 5 digits
                            msg_buf[14] <= "A";
                            msg_buf[15] <= "X";
                            msg_buf[16] <= ":";
                            msg_buf[17] <= lat_ax[15] ? "-" : "+";
                            msg_buf[18] <= 8'd48 + {4'd0, dig4};
                            msg_buf[19] <= 8'd48 + {4'd0, dig3};
                            msg_buf[20] <= 8'd48 + {4'd0, dig2};
                            msg_buf[21] <= 8'd48 + {4'd0, dig1};
                            msg_buf[22] <= 8'd48 + {4'd0, dig0};
                            msg_buf[23] <= " ";
                            build_step <= 4'd2;
                            abs_val <= lat_ay[15] ? (~lat_ay + 16'd1) : lat_ay;
                        end

                        4'd2: begin
                            // "AY:" + sign + 5 digits
                            msg_buf[24] <= "A";
                            msg_buf[25] <= "Y";
                            msg_buf[26] <= ":";
                            msg_buf[27] <= lat_ay[15] ? "-" : "+";
                            msg_buf[28] <= 8'd48 + {4'd0, dig4};
                            msg_buf[29] <= 8'd48 + {4'd0, dig3};
                            msg_buf[30] <= 8'd48 + {4'd0, dig2};
                            msg_buf[31] <= 8'd48 + {4'd0, dig1};
                            msg_buf[32] <= 8'd48 + {4'd0, dig0};
                            msg_buf[33] <= " ";
                            build_step <= 4'd3;
                            abs_val <= lat_az[15] ? (~lat_az + 16'd1) : lat_az;
                        end

                        4'd3: begin
                            // "AZ:" + sign + 5 digits + \r\n
                            msg_buf[34] <= "A";
                            msg_buf[35] <= "Z";
                            msg_buf[36] <= ":";
                            msg_buf[37] <= lat_az[15] ? "-" : "+";
                            msg_buf[38] <= 8'd48 + {4'd0, dig4};
                            msg_buf[39] <= 8'd48 + {4'd0, dig3};
                            msg_buf[40] <= 8'd48 + {4'd0, dig2};
                            msg_buf[41] <= 8'd48 + {4'd0, dig1};
                            msg_buf[42] <= 8'd48 + {4'd0, dig0};
                            msg_buf[43] <= 8'd13;  // \r
                            msg_buf[44] <= 8'd10;  // \n
                            msg_len     <= 6'd45;
                            msg_idx     <= 6'd0;
                            state       <= ST_SEND;
                        end
                    endcase
                end

                ST_SEND: begin
                    if (!tx_busy && msg_idx < msg_len) begin
                        tx_data  <= msg_buf[msg_idx];
                        tx_valid <= 1'b1;
                        msg_idx  <= msg_idx + 6'd1;
                        state    <= ST_WAIT;
                    end else if (msg_idx >= msg_len) begin
                        state <= ST_IDLE;
                    end
                end

                ST_WAIT: begin
                    // Wait for UART to finish sending byte
                    if (!tx_busy)
                        state <= ST_SEND;
                end
            endcase
        end
    end

endmodule
