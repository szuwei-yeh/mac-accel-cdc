`timescale 1ns/1ps

// mac_dma_rob.v
// -----------------------------------------------------------------------------
// Experimental V3 DMA front-end for the ROB/outstanding-transaction path.
//
// This wrapper preserves the existing V2 DMA contract: read vector A, then read
// vector B, and emit paired {last, a, b} samples to the async FIFO write side.
// The difference is that each operand phase is served by axi_read_engine_rob,
// which issues multiple unique-ID AXI reads and restores in-order phase output
// through a reorder buffer.
//
// This module is intentionally standalone and is not wired into mac_accel_dma_top.
// The verified V2 path remains rtl/mac_dma.v.
// -----------------------------------------------------------------------------

module mac_dma_rob #(
    parameter DATA_WIDTH      = 16,
    parameter AXI_DATA_WIDTH  = 32,
    parameter AXI_ADDR_WIDTH  = 32,
    parameter AXI_ID_WIDTH    = 4,
    parameter MAX_LEN         = 256,
    parameter BURST_LEN       = 16,
    parameter MAX_OUTSTANDING = 4
)(
    input  wire                       clk,
    input  wire                       rst,

    //----------------------------------------------------
    // Control
    //----------------------------------------------------
    input  wire                       start,
    input  wire [AXI_ADDR_WIDTH-1:0]  src_a_addr,
    input  wire [AXI_ADDR_WIDTH-1:0]  src_b_addr,
    input  wire [15:0]                length,
    output reg                        dma_busy,
    output reg                        dma_done,
    output reg                        dma_err,

    //----------------------------------------------------
    // AXI4 master AR channel
    //----------------------------------------------------
    output wire [AXI_ID_WIDTH-1:0]    M_AXI_ARID,
    output wire [AXI_ADDR_WIDTH-1:0]  M_AXI_ARADDR,
    output wire [7:0]                 M_AXI_ARLEN,
    output wire [2:0]                 M_AXI_ARSIZE,
    output wire [1:0]                 M_AXI_ARBURST,
    output wire                       M_AXI_ARVALID,
    input  wire                       M_AXI_ARREADY,

    //----------------------------------------------------
    // AXI4 master R channel
    //----------------------------------------------------
    input  wire [AXI_ID_WIDTH-1:0]    M_AXI_RID,
    input  wire [AXI_DATA_WIDTH-1:0]  M_AXI_RDATA,
    input  wire [1:0]                 M_AXI_RRESP,
    input  wire                       M_AXI_RLAST,
    input  wire                       M_AXI_RVALID,
    output wire                       M_AXI_RREADY,

    //----------------------------------------------------
    // Outgoing stream -> async FIFO write side
    //----------------------------------------------------
    output reg                          stream_valid,
    output reg                          stream_last,
    output reg  signed [DATA_WIDTH-1:0] stream_a,
    output reg  signed [DATA_WIDTH-1:0] stream_b,
    input  wire                         stream_ready
);

    localparam S_IDLE  = 3'd0;
    localparam S_CMD_A = 3'd1;
    localparam S_RUN_A = 3'd2;
    localparam S_CMD_B = 3'd3;
    localparam S_RUN_B = 3'd4;
    localparam S_DONE  = 3'd5;
    localparam BUF_IDX_W = (MAX_LEN <= 1) ? 1 : $clog2(MAX_LEN);

    reg [2:0] state;

    reg [AXI_ADDR_WIDTH-1:0] src_a_r;
    reg [AXI_ADDR_WIDTH-1:0] src_b_r;
    reg [15:0]               length_r;
    reg [15:0]               write_idx_a;
    reg [15:0]               read_idx_a;

    reg signed [DATA_WIDTH-1:0] buf_a [0:MAX_LEN-1];

    //----------------------------------------------------
    // ROB read engine command/output
    //----------------------------------------------------
    wire engine_cmd_valid = (state == S_CMD_A) || (state == S_CMD_B);
    wire engine_cmd_ready;
    wire [AXI_ADDR_WIDTH-1:0] engine_cmd_addr =
        (state == S_CMD_A) ? src_a_r : src_b_r;

    wire engine_out_valid;
    wire engine_out_ready;
    wire [AXI_DATA_WIDTH-1:0] engine_out_data;
    wire engine_out_last;
    wire engine_busy;
    wire engine_done;
    wire engine_err;

    assign engine_out_ready =
        (state == S_RUN_A) ? 1'b1 :
        (state == S_RUN_B) ? (!stream_valid || stream_ready) :
                             1'b0;

    wire engine_out_acc = engine_out_valid && engine_out_ready;
    wire a_out_acc = (state == S_RUN_A) && engine_out_acc;
    wire b_out_acc = (state == S_RUN_B) && engine_out_acc;

    axi_read_engine_rob #(
        .AXI_DATA_W      (AXI_DATA_WIDTH),
        .AXI_ADDR_W      (AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH    (AXI_ID_WIDTH),
        .MAX_OUTSTANDING (MAX_OUTSTANDING),
        .MAX_BURST_LEN   (BURST_LEN),
        .LEN_WIDTH       (16)
    ) u_read_engine (
        .clk             (clk),
        .rst             (rst),

        .cmd_valid       (engine_cmd_valid),
        .cmd_ready       (engine_cmd_ready),
        .cmd_addr        (engine_cmd_addr),
        .cmd_len         (length_r),

        .M_AXI_ARID      (M_AXI_ARID),
        .M_AXI_ARADDR    (M_AXI_ARADDR),
        .M_AXI_ARLEN     (M_AXI_ARLEN),
        .M_AXI_ARSIZE    (M_AXI_ARSIZE),
        .M_AXI_ARBURST   (M_AXI_ARBURST),
        .M_AXI_ARVALID   (M_AXI_ARVALID),
        .M_AXI_ARREADY   (M_AXI_ARREADY),

        .M_AXI_RID       (M_AXI_RID),
        .M_AXI_RDATA     (M_AXI_RDATA),
        .M_AXI_RRESP     (M_AXI_RRESP),
        .M_AXI_RLAST     (M_AXI_RLAST),
        .M_AXI_RVALID    (M_AXI_RVALID),
        .M_AXI_RREADY    (M_AXI_RREADY),

        .out_valid       (engine_out_valid),
        .out_ready       (engine_out_ready),
        .out_data        (engine_out_data),
        .out_last        (engine_out_last),

        .engine_busy     (engine_busy),
        .engine_done     (engine_done),
        .engine_err      (engine_err)
    );

    wire [BUF_IDX_W-1:0] write_idx_buf = write_idx_a[BUF_IDX_W-1:0];
    wire [BUF_IDX_W-1:0] read_idx_buf  = read_idx_a[BUF_IDX_W-1:0];

    wire _unused_ok = &{1'b0, engine_busy, engine_out_data[AXI_DATA_WIDTH-1:DATA_WIDTH]};

    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            src_a_r      <= {AXI_ADDR_WIDTH{1'b0}};
            src_b_r      <= {AXI_ADDR_WIDTH{1'b0}};
            length_r     <= 16'd0;
            write_idx_a  <= 16'd0;
            read_idx_a   <= 16'd0;
            dma_busy     <= 1'b0;
            dma_done     <= 1'b0;
            dma_err      <= 1'b0;
            stream_valid <= 1'b0;
            stream_last  <= 1'b0;
            stream_a     <= {DATA_WIDTH{1'b0}};
            stream_b     <= {DATA_WIDTH{1'b0}};
        end else begin
            dma_done <= 1'b0;

            if (engine_err)
                dma_err <= 1'b1;

            if (stream_valid && stream_ready)
                stream_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    dma_busy <= 1'b0;
                    if (start) begin
                        src_a_r      <= src_a_addr;
                        src_b_r      <= src_b_addr;
                        length_r     <= length;
                        write_idx_a  <= 16'd0;
                        read_idx_a   <= 16'd0;
                        dma_busy     <= 1'b1;
                        dma_err      <= 1'b0;
                        stream_valid <= 1'b0;
                        state        <= S_CMD_A;
                    end
                end

                S_CMD_A: begin
                    dma_busy <= 1'b1;
                    if (engine_cmd_ready)
                        state <= S_RUN_A;
                end

                S_RUN_A: begin
                    dma_busy <= 1'b1;
                    if (a_out_acc) begin
                        buf_a[write_idx_buf] <= engine_out_data[DATA_WIDTH-1:0];
                        write_idx_a        <= write_idx_a + 16'd1;
                    end
                    if (engine_done) begin
                        read_idx_a <= 16'd0;
                        state      <= S_CMD_B;
                    end
                end

                S_CMD_B: begin
                    dma_busy <= 1'b1;
                    if (engine_cmd_ready)
                        state <= S_RUN_B;
                end

                S_RUN_B: begin
                    dma_busy <= 1'b1;
                    if (b_out_acc) begin
                        stream_a     <= buf_a[read_idx_buf];
                        stream_b     <= engine_out_data[DATA_WIDTH-1:0];
                        stream_last  <= engine_out_last;
                        stream_valid <= 1'b1;
                        read_idx_a   <= read_idx_a + 16'd1;
                    end
                    if (engine_done)
                        state <= S_DONE;
                end

                S_DONE: begin
                    dma_done <= 1'b1;
                    dma_busy <= 1'b0;
                    state    <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

`ifndef SYNTHESIS
    initial begin
        $display("[MAC_DMA_ROB] instantiated: BURST_LEN=%0d MAX_OUTSTANDING=%0d MAX_LEN=%0d",
                 BURST_LEN, MAX_OUTSTANDING, MAX_LEN);
    end
`endif

endmodule
