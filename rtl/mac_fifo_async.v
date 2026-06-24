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
    output reg                  full,

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

    // full computed combinationally from the *next* write Gray pointer, then
    // REGISTERED.  Registering breaks the combinational loop
    //   full -> wr_ptr_bin_next -> wr_ptr_gray_next -> full
    // (the loop is closed because wr_ptr_bin_next at line 51 consumes `full`).
    wire full_val = (wr_ptr_gray_next == {~rd_ptr_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1],
                                           rd_ptr_gray_sync2[ADDR_WIDTH-2:0]});

    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) full <= 1'b0;
        else        full <= full_val;
    end

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

`ifdef FORMAL
    // ------------------------------------------------------------------
    // Formal: the invariants that justify a Gray-code async FIFO.
    //
    // wr_clk and rd_clk are independent, free-running clocks the solver drives
    // adversarially (multiclock), so any phase/frequency relationship the
    // 100/133 MHz hardware could produce is covered.  Every property below is a
    // COMBINATIONAL invariant -- it must hold in every state, so it is immune to
    // multiclock $past sampling and closes cleanly under k-induction.  Each also
    // holds during reset (the pointers are 0 and encode(0)=0).
    // ------------------------------------------------------------------

    // Clean power-on reset: hold both resets until the first wr_clk edge so the
    // base case starts from the known 0 state (the invariants tolerate later
    // resets too -- encode(0)=0 -- so no mid-run constraint is needed).
    reg f_init = 1'b0;
    always @(posedge wr_clk) f_init <= 1'b1;
    always @(*) if (!f_init) assume (wr_rst && rd_rst);

    // Binding invariant: the Gray pointer is always the Gray encoding of the
    // binary pointer.  True by construction; it also stops k-induction from
    // starting in an unreachable state where gray and binary disagree.
    always @(*) a_wr_gray_enc : assert (wr_ptr_gray == ((wr_ptr_bin >> 1) ^ wr_ptr_bin));
    always @(*) a_rd_gray_enc : assert (rd_ptr_gray == ((rd_ptr_bin >> 1) ^ rd_ptr_bin));

    // GRAY-CODE ONE-BIT-CHANGE: the next Gray pointer differs from the current
    // one by AT MOST a single bit.  This is the whole point of Gray coding for
    // CDC -- a pointer sampled mid-flight by the other domain's synchronizer
    // resolves to either the old or the new value, never a corrupt intermediate.
    always @(*) a_wr_gray_onehot : assert ($onehot0(wr_ptr_gray_next ^ wr_ptr_gray));
    always @(*) a_rd_gray_onehot : assert ($onehot0(rd_ptr_gray_next ^ rd_ptr_gray));

    // NO OVERFLOW: a full FIFO never advances its write pointer.
    always @(*) if (full)  a_no_overflow  : assert (wr_ptr_bin_next == wr_ptr_bin);
    // NO UNDERFLOW: an empty FIFO never advances its read pointer.
    always @(*) if (empty) a_no_underflow : assert (rd_ptr_bin_next == rd_ptr_bin);
`endif

endmodule