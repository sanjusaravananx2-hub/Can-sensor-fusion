//============================================================================
// can_sensor_axi_slave.v
// Avalon-MM Slave — CAN Sensor Register Interface
//
// Target:  Intel Cyclone V (DE1-SoC), 50 MHz system clock
// Purpose: Exposes CAN frame data, timestamps, raw and filtered
//          accelerometer values to the HPS via the lightweight HPS-to-FPGA
//          bridge (Avalon-MM, compatible with Qsys/Platform Designer).
//
// Register Map (word-aligned, 32-bit reads/writes):
//   Offset  Name          Access  Description
//   ------  -----------   ------  -----------
//   0x00    STATUS        R       [0] frame_valid, [1] error, [15:8] err_count[7:0]
//   0x01    CAN_ID        R       [10:0] standard CAN ID of last frame
//   0x02    CAN_DATA_LO   R       data bytes 0-3 (MSB = byte 0)
//   0x03    CAN_DATA_HI   R       data bytes 4-7
//   0x04    TIMESTAMP     R       32-bit microsecond timestamp of last frame
//   0x05    ACCEL_RAW_X   R       [15:0] signed raw accelerometer X
//   0x06    ACCEL_RAW_Y   R       [15:0] signed raw accelerometer Y
//   0x07    ACCEL_RAW_Z   R       [15:0] signed raw accelerometer Z
//   0x08    ACCEL_FILT_X  R       [31:0] signed filtered accelerometer X
//   0x09    ACCEL_FILT_Y  R       [31:0] signed filtered accelerometer Y
//   0x0A    ACCEL_FILT_Z  R       [31:0] signed filtered accelerometer Z
//   0x0C    FIR_COEFF     W       {[18:16] coeff_addr, [15:0] coeff_data}
//   0x10    FRAME_COUNT   R       32-bit total frame counter
//   0x11    ERROR_COUNT   R       32-bit error counter
//
// Internal logic:
//   - Detects CAN ID 0x100 as accelerometer frame
//   - Parses 6 bytes: X[15:0] = {D0,D1}, Y = {D2,D3}, Z = {D4,D5}
//   - Routes each axis through an fir_filter instance
//   - Instantiates hw_timestamp for frame timestamping
//============================================================================

module can_sensor_axi_slave (
    input  wire        clk,
    input  wire        rst_n,

    // Avalon-MM slave interface
    input  wire [4:0]  address,       // word address (5 bits for up to 32 regs)
    input  wire [31:0] writedata,
    output reg  [31:0] readdata,
    input  wire        read,
    input  wire        write,
    input  wire        chipselect,

    // CAN frame input (from mcp2515_controller)
    input  wire [10:0] can_id,
    input  wire [63:0] can_data,
    input  wire [3:0]  can_dlc,
    input  wire        can_valid,
    input  wire        can_error,

    // Filtered accel outputs (exposed for LED display)
    output wire signed [15:0] accel_raw_x,
    output wire signed [15:0] accel_raw_y,
    output wire signed [15:0] accel_raw_z,

    // Frame and error counters (exposed for LED display)
    output wire [31:0] frame_count_out,
    output wire [31:0] error_count_out,

    // Timestamp output
    output wire [31:0] timestamp_out
);

    // ----------------------------------------------------------------
    // Registers — latched CAN frame data
    // ----------------------------------------------------------------
    reg        reg_frame_valid;
    reg        reg_error;
    reg [10:0] reg_can_id;
    reg [63:0] reg_can_data;
    reg [31:0] reg_timestamp;
    reg [31:0] frame_count;
    reg [31:0] error_count;

    // Accelerometer raw values (extracted from CAN ID 0x100)
    reg signed [15:0] reg_accel_raw_x;
    reg signed [15:0] reg_accel_raw_y;
    reg signed [15:0] reg_accel_raw_z;

    // Accelerometer filtered values (from FIR outputs)
    wire signed [31:0] accel_filt_x;
    wire signed [31:0] accel_filt_y;
    wire signed [31:0] accel_filt_z;

    // Expose raw accel for LED module
    assign accel_raw_x     = reg_accel_raw_x;
    assign accel_raw_y     = reg_accel_raw_y;
    assign accel_raw_z     = reg_accel_raw_z;
    assign frame_count_out = frame_count;
    assign error_count_out = error_count;

    // ----------------------------------------------------------------
    // Timestamp module
    // ----------------------------------------------------------------
    wire [31:0] ts_value;
    wire        ts_valid;

    hw_timestamp u_timestamp (
        .clk            (clk),
        .rst_n          (rst_n),
        .can_valid      (can_valid),
        .timestamp_out  (ts_value),
        .timestamp_valid(ts_valid)
    );

    assign timestamp_out = reg_timestamp;

    // ----------------------------------------------------------------
    // FIR coefficient write interface
    // ----------------------------------------------------------------
    reg        fir_coeff_wr;
    reg [2:0]  fir_coeff_addr;
    reg [15:0] fir_coeff_data;

    // Accel valid pulse — triggers FIR input when accel frame arrives
    reg        accel_valid;

    // ----------------------------------------------------------------
    // FIR filter instances (X, Y, Z)
    // ----------------------------------------------------------------
    wire fir_valid_x, fir_valid_y, fir_valid_z;

    fir_filter u_fir_x (
        .clk           (clk),
        .rst_n         (rst_n),
        .coeff_wr      (fir_coeff_wr),
        .coeff_addr    (fir_coeff_addr),
        .coeff_data    (fir_coeff_data),
        .data_in       (reg_accel_raw_x),
        .data_valid    (accel_valid),
        .data_out      (accel_filt_x),
        .data_out_valid(fir_valid_x)
    );

    fir_filter u_fir_y (
        .clk           (clk),
        .rst_n         (rst_n),
        .coeff_wr      (fir_coeff_wr),
        .coeff_addr    (fir_coeff_addr),
        .coeff_data    (fir_coeff_data),
        .data_in       (reg_accel_raw_y),
        .data_valid    (accel_valid),
        .data_out      (accel_filt_y),
        .data_out_valid(fir_valid_y)
    );

    fir_filter u_fir_z (
        .clk           (clk),
        .rst_n         (rst_n),
        .coeff_wr      (fir_coeff_wr),
        .coeff_addr    (fir_coeff_addr),
        .coeff_data    (fir_coeff_data),
        .data_in       (reg_accel_raw_z),
        .data_valid    (accel_valid),
        .data_out      (accel_filt_z),
        .data_out_valid(fir_valid_z)
    );

    // ----------------------------------------------------------------
    // CAN frame processing — latch data and extract accelerometer
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            reg_frame_valid <= 1'b0;
            reg_error       <= 1'b0;
            reg_can_id      <= 11'd0;
            reg_can_data    <= 64'd0;
            reg_timestamp   <= 32'd0;
            frame_count     <= 32'd0;
            error_count     <= 32'd0;
            reg_accel_raw_x <= 16'sd0;
            reg_accel_raw_y <= 16'sd0;
            reg_accel_raw_z <= 16'sd0;
            accel_valid     <= 1'b0;
        end else begin
            accel_valid <= 1'b0;    // default: clear pulse

            // Latch frame data on can_valid
            if (can_valid) begin
                reg_frame_valid <= 1'b1;
                reg_can_id      <= can_id;
                reg_can_data    <= can_data;
                frame_count     <= frame_count + 32'd1;

                // Extract accelerometer data from CAN ID 0x100
                // Frame format: D0:D1 = X, D2:D3 = Y, D4:D5 = Z (big-endian signed)
                if (can_id == 11'h100) begin
                    reg_accel_raw_x <= $signed({can_data[63:56], can_data[55:48]});
                    reg_accel_raw_y <= $signed({can_data[47:40], can_data[39:32]});
                    reg_accel_raw_z <= $signed({can_data[31:24], can_data[23:16]});
                    accel_valid     <= 1'b1;
                end
            end

            // Latch timestamp
            if (ts_valid) begin
                reg_timestamp <= ts_value;
            end

            // Error tracking
            if (can_error) begin
                reg_error   <= 1'b1;
                error_count <= error_count + 32'd1;
            end

            // Clear frame_valid when HPS reads STATUS register
            if (chipselect && read && address == 5'd0) begin
                reg_frame_valid <= 1'b0;
            end
        end
    end

    // ----------------------------------------------------------------
    // Avalon-MM write handling — FIR coefficient writes
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            fir_coeff_wr   <= 1'b0;
            fir_coeff_addr <= 3'd0;
            fir_coeff_data <= 16'd0;
        end else begin
            fir_coeff_wr <= 1'b0;  // default

            if (chipselect && write) begin
                case (address)
                    5'd12: begin   // FIR_COEFF register
                        fir_coeff_wr   <= 1'b1;
                        fir_coeff_addr <= writedata[18:16];
                        fir_coeff_data <= writedata[15:0];
                    end
                    default: ; // other addresses are read-only
                endcase
            end
        end
    end

    // ----------------------------------------------------------------
    // Avalon-MM read mux
    // ----------------------------------------------------------------
    always @(*) begin
        readdata = 32'd0;
        if (chipselect && read) begin
            case (address)
                5'd0:  readdata = {16'd0, error_count[7:0], 6'd0, reg_error, reg_frame_valid};
                5'd1:  readdata = {21'd0, reg_can_id};
                5'd2:  readdata = reg_can_data[63:32];   // CAN_DATA_LO (bytes 0-3)
                5'd3:  readdata = reg_can_data[31:0];    // CAN_DATA_HI (bytes 4-7)
                5'd4:  readdata = reg_timestamp;
                5'd5:  readdata = {{16{reg_accel_raw_x[15]}}, reg_accel_raw_x};
                5'd6:  readdata = {{16{reg_accel_raw_y[15]}}, reg_accel_raw_y};
                5'd7:  readdata = {{16{reg_accel_raw_z[15]}}, reg_accel_raw_z};
                5'd8:  readdata = accel_filt_x;
                5'd9:  readdata = accel_filt_y;
                5'd10: readdata = accel_filt_z;
                5'd16: readdata = frame_count;
                5'd17: readdata = error_count;
                default: readdata = 32'd0;
            endcase
        end
    end

endmodule
