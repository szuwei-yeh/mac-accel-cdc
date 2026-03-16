`timescale 1ns/1ps

// mac_pe.v
// 2-stage pipelined MAC PE
// - operates in mac_clk domain
// - reads {last, a, b} from Async FIFO

module mac_pe #(
    parameter DATA_WIDTH = 16
)(
    input  wire                     clk,        // mac_clk
    input  wire                     rst,        // mac_rst

    // Control signals (from bus domain via CDC)
    input  wire                     start,      // pulse: start new task
    output reg                      busy,       // MAC running
    output reg                      done,       // pulse: complete

    // Read data from FIFO
    input  wire [2*DATA_WIDTH:0]    fifo_dout,  // {last, a, b}
    input  wire                     fifo_empty,
    output reg                      fifo_rd_en,

    // Result output (mac_clk domain)
    output reg  signed [2*DATA_WIDTH-1:0]  result,
    output reg  [31:0]                     last_latency
);

    // Unpack FIFO data
    wire                          fifo_last;
    wire signed [DATA_WIDTH-1:0]  fifo_a;
    wire signed [DATA_WIDTH-1:0]  fifo_b;

    assign {fifo_last, fifo_a, fifo_b} = fifo_dout;

    // FSM
    localparam S_IDLE = 1'b0;
    localparam S_RUN  = 1'b1;
    reg state;

    // 2-stage pipeline
    // [FIX] Use signed types to ensure correct signed multiplication and accumulation
    reg signed [2*DATA_WIDTH-1:0] mul_reg;
    reg                            valid_mul;
    reg                            last_mul;
    reg signed [2*DATA_WIDTH-1:0] acc_reg;

    reg [31:0] cycle_cnt;

    // Read FIFO -> generate internal valid / data
    reg                          valid_in_r;
    // [FIX] Use signed types to prevent negative values being treated as large positives
    reg signed [DATA_WIDTH-1:0]  a_in_r;
    reg signed [DATA_WIDTH-1:0]  b_in_r;
    reg                          last_in_r;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            fifo_rd_en  <= 1'b0;
            valid_in_r  <= 1'b0;
            a_in_r      <= 0;
            b_in_r      <= 0;
            last_in_r   <= 1'b0;
        end else begin
            fifo_rd_en  <= 1'b0;
            valid_in_r  <= 1'b0;
            last_in_r   <= 1'b0;

            if (state == S_RUN && !fifo_empty && !fifo_rd_en) begin
                fifo_rd_en <= 1'b1;
                valid_in_r <= 1'b1;
                a_in_r     <= fifo_a;
                b_in_r     <= fifo_b;
                last_in_r  <= fifo_last;
            end
        end
    end

    // Main FSM + pipeline
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= S_IDLE;
            busy         <= 1'b0;
            done         <= 1'b0;
            mul_reg      <= 0;
            valid_mul    <= 1'b0;
            last_mul     <= 1'b0;
            acc_reg      <= 0;
            result       <= 0;
            cycle_cnt    <= 0;
            last_latency <= 0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy      <= 1'b0;
                    valid_mul <= 1'b0;
                    last_mul  <= 1'b0;
                    cycle_cnt <= 0;

                    if (start) begin
                        acc_reg <= 0;
                        result  <= 0;
                        busy    <= 1'b1;
                        state   <= S_RUN;
                    end
                end

                S_RUN: begin
                    busy      <= 1'b1;
                    cycle_cnt <= cycle_cnt + 1;

                    // stage1: mul (signed x signed = signed)
                    if (valid_in_r) begin
                        mul_reg   <= a_in_r * b_in_r;
                        valid_mul <= 1'b1;
                        last_mul  <= last_in_r;
                    end else begin
                        valid_mul <= 1'b0;
                        last_mul  <= 1'b0;
                    end

                    // stage2: accumulate
                    if (valid_mul) begin
                        acc_reg <= acc_reg + mul_reg;

                        if (last_mul) begin
                            result       <= acc_reg + mul_reg;
                            done         <= 1'b1;
                            busy         <= 1'b0;
                            state        <= S_IDLE;
                            last_latency <= cycle_cnt + 1;
                        end
                    end
                end
            endcase
        end
    end

endmodule