`timescale 1ns/1ps

// mac_accel_dma_top.v
// Top-level MAC accelerator with AXI4 master DMA front-end (V2).
//
// V1 (mac_accel_axi.v) required the CPU to write every element via MMIO
// (AIN/BIN registers).  V2 keeps only a small control surface on the
// AXI4-Lite slave -- the accelerator now burst-reads its operands directly
// from system memory.
//
// Register map (S_AXI, 4-byte aligned):
//   0x00 CTRL        W:bit[0]=start  R:{29'b0, dma_err, done, busy, 1'b0}
//   0x04 SRC_A_ADDR  W: vector A base address (4-byte aligned)
//   0x08 SRC_B_ADDR  W: vector B base address (4-byte aligned)
//   0x0C LENGTH      W: element count per vector (1..MAX_LEN)
//   0x10 RESULT      R: signed dot-product result (latched on done)
//   0x14 LATENCY     R: mac_clk cycle count from start to done
//
// Two clock domains:
//   bus_clk: S_AXI slave, M_AXI master, mac_dma, register file
//   mac_clk: mac_pe, accumulation pipeline
// Async FIFO crosses bus_clk -> mac_clk for the {last, a, b} stream.

module mac_accel_dma_top #(
    parameter DATA_WIDTH      = 16,
    parameter AXI_DATA_WIDTH  = 32,
    parameter AXI_ADDR_WIDTH  = 32,
    parameter AXI_ID_WIDTH    = 4,
    parameter S_AXI_ADDR_WIDTH = 8,
    parameter MAX_LEN         = 256,
    parameter BURST_LEN       = 16
)(
    //----------------------------------------------------
    // AXI4-Lite slave (control plane) - bus_clk domain
    //----------------------------------------------------
    input  wire                          S_AXI_ACLK,
    input  wire                          S_AXI_ARESETN,

    input  wire [S_AXI_ADDR_WIDTH-1:0]   S_AXI_AWADDR,
    input  wire [2:0]                    S_AXI_AWPROT,
    input  wire                          S_AXI_AWVALID,
    output reg                           S_AXI_AWREADY,

    input  wire [AXI_DATA_WIDTH-1:0]     S_AXI_WDATA,
    input  wire [AXI_DATA_WIDTH/8-1:0]   S_AXI_WSTRB,
    input  wire                          S_AXI_WVALID,
    output reg                           S_AXI_WREADY,

    output reg  [1:0]                    S_AXI_BRESP,
    output reg                           S_AXI_BVALID,
    input  wire                          S_AXI_BREADY,

    input  wire [S_AXI_ADDR_WIDTH-1:0]   S_AXI_ARADDR,
    input  wire [2:0]                    S_AXI_ARPROT,
    input  wire                          S_AXI_ARVALID,
    output reg                           S_AXI_ARREADY,

    output reg  [AXI_DATA_WIDTH-1:0]     S_AXI_RDATA,
    output reg  [1:0]                    S_AXI_RRESP,
    output reg                           S_AXI_RVALID,
    input  wire                          S_AXI_RREADY,

    //----------------------------------------------------
    // AXI4 master (data plane / DMA) - bus_clk domain
    //----------------------------------------------------
    output wire [AXI_ID_WIDTH-1:0]       M_AXI_ARID,
    output wire [AXI_ADDR_WIDTH-1:0]     M_AXI_ARADDR,
    output wire [7:0]                    M_AXI_ARLEN,
    output wire [2:0]                    M_AXI_ARSIZE,
    output wire [1:0]                    M_AXI_ARBURST,
    output wire                          M_AXI_ARVALID,
    input  wire                          M_AXI_ARREADY,

    input  wire [AXI_ID_WIDTH-1:0]       M_AXI_RID,
    input  wire [AXI_DATA_WIDTH-1:0]     M_AXI_RDATA,
    input  wire [1:0]                    M_AXI_RRESP,
    input  wire                          M_AXI_RLAST,
    input  wire                          M_AXI_RVALID,
    output wire                          M_AXI_RREADY,

    //----------------------------------------------------
    // MAC core clock domain
    //----------------------------------------------------
    input  wire                          mac_clk,
    input  wire                          mac_rst,    // active-high

    //----------------------------------------------------
    // Debug
    //----------------------------------------------------
    output wire                          done_led
);

    //----------------------------------------------------
    // Aliases
    //----------------------------------------------------
    wire bus_clk = S_AXI_ACLK;
    wire bus_rst = ~S_AXI_ARESETN;

    //----------------------------------------------------
    // Register file (bus_clk domain)
    //----------------------------------------------------
    localparam ADDR_CTRL    = 4'h0;
    localparam ADDR_SRC_A   = 4'h1;
    localparam ADDR_SRC_B   = 4'h2;
    localparam ADDR_LENGTH  = 4'h3;
    localparam ADDR_RESULT  = 4'h4;
    localparam ADDR_LATENCY = 4'h5;

    reg [AXI_ADDR_WIDTH-1:0] reg_src_a;
    reg [AXI_ADDR_WIDTH-1:0] reg_src_b;
    reg [15:0]               reg_length;

    //----------------------------------------------------
    // AXI4-Lite write FSM
    //   - Latch AW and W independently, fire when both held
    //----------------------------------------------------
    reg                          aw_latched;
    reg [3:0]                    aw_word_addr;
    reg                          w_latched;
    reg [AXI_DATA_WIDTH-1:0]     w_data_r;
    reg [AXI_DATA_WIDTH/8-1:0]   w_strb_r;

    wire [3:0] wr_word_addr = S_AXI_AWADDR[5:2];
    wire [3:0] rd_word_addr = S_AXI_ARADDR[5:2];

    // start pulse: 1 bus_clk cycle, generated on CTRL[0] write
    reg start_pulse_bus;

    always @(posedge bus_clk) begin
        if (bus_rst) begin
            S_AXI_AWREADY <= 1'b0;
            S_AXI_WREADY  <= 1'b0;
            S_AXI_BVALID  <= 1'b0;
            S_AXI_BRESP   <= 2'b00;
            aw_latched    <= 1'b0;
            w_latched     <= 1'b0;
            aw_word_addr  <= 4'd0;
            w_data_r      <= {AXI_DATA_WIDTH{1'b0}};
            w_strb_r      <= {AXI_DATA_WIDTH/8{1'b0}};
            reg_src_a     <= {AXI_ADDR_WIDTH{1'b0}};
            reg_src_b     <= {AXI_ADDR_WIDTH{1'b0}};
            reg_length    <= 16'd0;
            start_pulse_bus <= 1'b0;
        end else begin
            start_pulse_bus <= 1'b0;
            S_AXI_AWREADY   <= 1'b0;
            S_AXI_WREADY    <= 1'b0;

            // AW accept
            if (S_AXI_AWVALID && !aw_latched) begin
                aw_word_addr  <= wr_word_addr;
                aw_latched    <= 1'b1;
                S_AXI_AWREADY <= 1'b1;
            end

            // W accept
            if (S_AXI_WVALID && !w_latched) begin
                w_data_r     <= S_AXI_WDATA;
                w_strb_r     <= S_AXI_WSTRB;
                w_latched    <= 1'b1;
                S_AXI_WREADY <= 1'b1;
            end

            // Both held -> commit to register file
            if (aw_latched && w_latched && !S_AXI_BVALID) begin
                case (aw_word_addr)
                    ADDR_CTRL: begin
                        if (w_data_r[0]) start_pulse_bus <= 1'b1;
                    end
                    ADDR_SRC_A:  reg_src_a  <= w_data_r[AXI_ADDR_WIDTH-1:0];
                    ADDR_SRC_B:  reg_src_b  <= w_data_r[AXI_ADDR_WIDTH-1:0];
                    ADDR_LENGTH: reg_length <= w_data_r[15:0];
                    default: ;
                endcase
                aw_latched   <= 1'b0;
                w_latched    <= 1'b0;
                S_AXI_BVALID <= 1'b1;
                S_AXI_BRESP  <= 2'b00;
            end

            if (S_AXI_BVALID && S_AXI_BREADY)
                S_AXI_BVALID <= 1'b0;
        end
    end

    //----------------------------------------------------
    // mac_dma instance (AXI4 master)
    //----------------------------------------------------
    wire                          dma_busy;
    wire                          dma_done_pulse;     // single-cycle pulse (bus_clk)
    wire                          dma_err;

    wire                          stream_valid;
    wire                          stream_last;
    wire signed [DATA_WIDTH-1:0]  stream_a;
    wire signed [DATA_WIDTH-1:0]  stream_b;
    wire                          fifo_full_bus;
    wire                          stream_ready = ~fifo_full_bus;

    mac_dma #(
        .DATA_WIDTH    (DATA_WIDTH),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH  (AXI_ID_WIDTH),
        .MAX_LEN       (MAX_LEN),
        .BURST_LEN     (BURST_LEN)
    ) u_dma (
        .clk            (bus_clk),
        .rst            (bus_rst),

        .start          (start_pulse_bus),
        .src_a_addr     (reg_src_a),
        .src_b_addr     (reg_src_b),
        .length         (reg_length),
        .dma_busy       (dma_busy),
        .dma_done       (dma_done_pulse),
        .dma_err        (dma_err),

        .M_AXI_ARID     (M_AXI_ARID),
        .M_AXI_ARADDR   (M_AXI_ARADDR),
        .M_AXI_ARLEN    (M_AXI_ARLEN),
        .M_AXI_ARSIZE   (M_AXI_ARSIZE),
        .M_AXI_ARBURST  (M_AXI_ARBURST),
        .M_AXI_ARVALID  (M_AXI_ARVALID),
        .M_AXI_ARREADY  (M_AXI_ARREADY),

        .M_AXI_RID      (M_AXI_RID),
        .M_AXI_RDATA    (M_AXI_RDATA),
        .M_AXI_RRESP    (M_AXI_RRESP),
        .M_AXI_RLAST    (M_AXI_RLAST),
        .M_AXI_RVALID   (M_AXI_RVALID),
        .M_AXI_RREADY   (M_AXI_RREADY),

        .stream_valid   (stream_valid),
        .stream_last    (stream_last),
        .stream_a       (stream_a),
        .stream_b       (stream_b),
        .stream_ready   (stream_ready)
    );

    //----------------------------------------------------
    // Async FIFO: bus_clk -> mac_clk
    // payload = {last, a, b}
    //----------------------------------------------------
    localparam FIFO_WIDTH     = 2*DATA_WIDTH + 1;
    localparam FIFO_ADDR_BITS = 4;                  // depth = 16

    wire [FIFO_WIDTH-1:0] fifo_din_bus  = {stream_last, stream_a, stream_b};
    wire                  fifo_wr_en    = stream_valid & ~fifo_full_bus;

    wire [FIFO_WIDTH-1:0] fifo_dout_mac;
    wire                  fifo_empty_mac;
    wire                  fifo_rd_en_mac;

    mac_fifo_async #(
        .WIDTH     (FIFO_WIDTH),
        .ADDR_WIDTH(FIFO_ADDR_BITS)
    ) u_fifo (
        .wr_clk (bus_clk),
        .wr_rst (bus_rst),
        .wr_en  (fifo_wr_en),
        .din    (fifo_din_bus),
        .full   (fifo_full_bus),

        .rd_clk (mac_clk),
        .rd_rst (mac_rst),
        .rd_en  (fifo_rd_en_mac),
        .dout   (fifo_dout_mac),
        .empty  (fifo_empty_mac)
    );

    //----------------------------------------------------
    // CDC: start (bus -> mac) via toggle sync
    //----------------------------------------------------
    reg start_toggle_bus;
    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst)              start_toggle_bus <= 1'b0;
        else if (start_pulse_bus) start_toggle_bus <= ~start_toggle_bus;
    end

    reg [1:0] start_sync_mac;
    always @(posedge mac_clk or posedge mac_rst) begin
        if (mac_rst) start_sync_mac <= 2'b00;
        else         start_sync_mac <= {start_sync_mac[0], start_toggle_bus};
    end

    reg start_sync_mac_d;
    always @(posedge mac_clk or posedge mac_rst) begin
        if (mac_rst) start_sync_mac_d <= 1'b0;
        else         start_sync_mac_d <= start_sync_mac[1];
    end
    wire start_pulse_mac = start_sync_mac[1] ^ start_sync_mac_d;

    //----------------------------------------------------
    // mac_pe instance (mac_clk domain)
    //----------------------------------------------------
    wire                          busy_mac;
    wire                          done_mac;
    wire signed [2*DATA_WIDTH-1:0] result_mac;
    wire [31:0]                    latency_mac;

    mac_pe #(
        .DATA_WIDTH(DATA_WIDTH)
    ) u_pe (
        .clk         (mac_clk),
        .rst         (mac_rst),
        .start       (start_pulse_mac),
        .busy        (busy_mac),
        .done        (done_mac),
        .fifo_dout   (fifo_dout_mac),
        .fifo_empty  (fifo_empty_mac),
        .fifo_rd_en  (fifo_rd_en_mac),
        .result      (result_mac),
        .last_latency(latency_mac)
    );

    //----------------------------------------------------
    // CDC: busy (mac -> bus), 2-FF sync
    //----------------------------------------------------
    reg [1:0] busy_sync_bus;
    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst) busy_sync_bus <= 2'b00;
        else         busy_sync_bus <= {busy_sync_bus[0], busy_mac};
    end
    wire busy_bus = busy_sync_bus[1];

    //----------------------------------------------------
    // CDC: done (mac -> bus) via toggle sync + edge detect
    //----------------------------------------------------
    reg done_toggle_mac;
    always @(posedge mac_clk or posedge mac_rst) begin
        if (mac_rst)       done_toggle_mac <= 1'b0;
        else if (done_mac) done_toggle_mac <= ~done_toggle_mac;
    end

    reg [1:0] done_sync_bus;
    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst) done_sync_bus <= 2'b00;
        else         done_sync_bus <= {done_sync_bus[0], done_toggle_mac};
    end

    reg done_sync_bus_d;
    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst) done_sync_bus_d <= 1'b0;
        else         done_sync_bus_d <= done_sync_bus[1];
    end
    wire done_pulse_bus = done_sync_bus[1] ^ done_sync_bus_d;

    // Sticky done flag (cleared on new start)
    reg done_bus;
    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst)                done_bus <= 1'b0;
        else if (start_pulse_bus)   done_bus <= 1'b0;
        else if (done_pulse_bus)    done_bus <= 1'b1;
    end
    assign done_led = done_bus;

    //----------------------------------------------------
    // Latch result/latency on done_pulse_bus
    //----------------------------------------------------
    reg [31:0] res_bus_reg;
    reg [31:0] lat_bus_reg;

    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst) begin
            res_bus_reg <= 32'd0;
            lat_bus_reg <= 32'd0;
        end else if (done_pulse_bus) begin
            res_bus_reg <= $signed(result_mac[2*DATA_WIDTH-1:0]);
            lat_bus_reg <= latency_mac;
        end
    end

    //----------------------------------------------------
    // Sticky DMA error (cleared on new start)
    //----------------------------------------------------
    reg dma_err_bus;
    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst)                dma_err_bus <= 1'b0;
        else if (start_pulse_bus)   dma_err_bus <= 1'b0;
        else if (dma_err)           dma_err_bus <= 1'b1;
    end

    //----------------------------------------------------
    // AXI4-Lite read FSM
    //----------------------------------------------------
    reg rd_pending;
    reg [3:0] ar_word_addr;

    reg [AXI_DATA_WIDTH-1:0] rd_mux;
    always @(*) begin
        case (ar_word_addr)
            ADDR_CTRL:    rd_mux = {28'b0, dma_err_bus, done_bus, busy_bus, 1'b0};
            ADDR_SRC_A:   rd_mux = reg_src_a;
            ADDR_SRC_B:   rd_mux = reg_src_b;
            ADDR_LENGTH:  rd_mux = {16'b0, reg_length};
            ADDR_RESULT:  rd_mux = res_bus_reg;
            ADDR_LATENCY: rd_mux = lat_bus_reg;
            default:      rd_mux = 32'hDEAD_BEEF;
        endcase
    end

    always @(posedge bus_clk) begin
        if (bus_rst) begin
            S_AXI_ARREADY <= 1'b0;
            S_AXI_RVALID  <= 1'b0;
            S_AXI_RRESP   <= 2'b00;
            S_AXI_RDATA   <= {AXI_DATA_WIDTH{1'b0}};
            rd_pending    <= 1'b0;
            ar_word_addr  <= 4'd0;
        end else begin
            S_AXI_ARREADY <= 1'b0;

            if (S_AXI_ARVALID && !rd_pending && !S_AXI_RVALID) begin
                ar_word_addr  <= rd_word_addr;
                rd_pending    <= 1'b1;
                S_AXI_ARREADY <= 1'b1;
            end

            if (rd_pending) begin
                S_AXI_RDATA  <= rd_mux;
                S_AXI_RRESP  <= 2'b00;
                S_AXI_RVALID <= 1'b1;
                rd_pending   <= 1'b0;
            end

            if (S_AXI_RVALID && S_AXI_RREADY)
                S_AXI_RVALID <= 1'b0;
        end
    end

    // tie off unused signal
    wire _unused_ok = &{1'b0, dma_busy, dma_done_pulse, S_AXI_AWPROT, S_AXI_ARPROT,
                       S_AXI_AWADDR[7:6], S_AXI_AWADDR[1:0],
                       S_AXI_ARADDR[7:6], S_AXI_ARADDR[1:0],
                       w_strb_r, M_AXI_RID};

`ifndef SYNTHESIS
    initial begin
        $display("[MAC_ACCEL_DMA_TOP] V2 (DMA) instantiated, time=%0t", $time);
    end
`endif

endmodule
