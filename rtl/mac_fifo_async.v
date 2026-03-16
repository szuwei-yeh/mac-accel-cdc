`timescale 1ns/1ps

// --------------------------------------------------------------------
// Async FIFO (dual-clock)
// - write side:  wr_clk, wr_rst
// - read  side:  rd_clk, rd_rst
// - uses binary pointer + Gray code + 2-FF synchronizer
// - dout: asynchronous read (combinatorial output)
// --------------------------------------------------------------------
module mac_fifo_async #(
    parameter WIDTH      = 32,
    parameter ADDR_WIDTH = 4   // DEPTH = 2^ADDR_WIDTH
)(
    // write domain
    input  wire                 wr_clk,
    input  wire                 wr_rst,
    input  wire                 wr_en,
    input  wire [WIDTH-1:0]     din,
    output wire                 full,

    // read domain
    input  wire                 rd_clk,
    input  wire                 rd_rst,
    input  wire                 rd_en,
    output wire [WIDTH-1:0]     dout,   // async read
    output wire                 empty
);

    localparam DEPTH = (1 << ADDR_WIDTH);

    // storage
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    // write pointer (binary / Gray)
    reg [ADDR_WIDTH:0] wr_ptr_bin;
    reg [ADDR_WIDTH:0] wr_ptr_gray;

    // rd_ptr_gray synchronized to write domain
    reg [ADDR_WIDTH:0] rd_ptr_gray_sync1;
    reg [ADDR_WIDTH:0] rd_ptr_gray_sync2;

    // read pointer (binary / Gray)
    reg [ADDR_WIDTH:0] rd_ptr_bin;
    reg [ADDR_WIDTH:0] rd_ptr_gray;

    // wr_ptr_gray synchronized to read domain
    reg [ADDR_WIDTH:0] wr_ptr_gray_sync1;
    reg [ADDR_WIDTH:0] wr_ptr_gray_sync2;

    // write logic
    wire [ADDR_WIDTH:0] wr_ptr_bin_next  = wr_ptr_bin + (wr_en && !full);
    wire [ADDR_WIDTH:0] wr_ptr_gray_next = (wr_ptr_bin_next >> 1) ^ wr_ptr_bin_next;

    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            wr_ptr_bin  <= 0;
            wr_ptr_gray <= 0;
        end else begin
            wr_ptr_bin  <= wr_ptr_bin_next;
            wr_ptr_gray <= wr_ptr_gray_next;
            if (wr_en && !full)
                mem[wr_ptr_bin[ADDR_WIDTH-1:0]] <= din;
        end
    end

    // sync rd_ptr_gray to write domain
    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            rd_ptr_gray_sync1 <= 0;
            rd_ptr_gray_sync2 <= 0;
        end else begin
            rd_ptr_gray_sync1 <= rd_ptr_gray;
            rd_ptr_gray_sync2 <= rd_ptr_gray_sync1;
        end
    end

    assign full = (wr_ptr_gray_next == {~rd_ptr_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1],
                                         rd_ptr_gray_sync2[ADDR_WIDTH-2:0]});

    // read logic - pointer only, no registered dout
    wire [ADDR_WIDTH:0] rd_ptr_bin_next  = rd_ptr_bin + (rd_en && !empty);
    wire [ADDR_WIDTH:0] rd_ptr_gray_next = (rd_ptr_bin_next >> 1) ^ rd_ptr_bin_next;

    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            rd_ptr_bin  <= 0;
            rd_ptr_gray <= 0;
        end else begin
            rd_ptr_bin  <= rd_ptr_bin_next;
            rd_ptr_gray <= rd_ptr_gray_next;
        end
    end

    // Asynchronous read output
    assign dout = mem[rd_ptr_bin[ADDR_WIDTH-1:0]];

    // sync wr_ptr_gray to read domain
    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            wr_ptr_gray_sync1 <= 0;
            wr_ptr_gray_sync2 <= 0;
        end else begin
            wr_ptr_gray_sync1 <= wr_ptr_gray;
            wr_ptr_gray_sync2 <= wr_ptr_gray_sync1;
        end
    end

    // empty check
    assign empty = (rd_ptr_gray == wr_ptr_gray_sync2);

endmodule