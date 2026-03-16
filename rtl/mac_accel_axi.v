`timescale 1ns/1ps

// mac_accel_axi.v
// AXI4-Lite slave wrapper for mac_accel
//
// Address map (word-aligned, 4-byte offset):
//   0x00  CTRL    W: bit[0]=start  R: bit[2]=done, bit[1]=busy
//   0x04  AIN     W: write next element of vecA (auto-increment)
//   0x08  BIN     W: write next element of vecB (auto-increment)
//   0x0C  MASK    W: update_mask[VEC_LEN-1:0], resets load index
//   0x10  RESULT  R: signed dot-product result
//   0x14  LATENCY R: cycle count (mac_clk) from start to done

module mac_accel_axi #(
    parameter DATA_WIDTH    = 16,
    parameter VEC_LEN       = 4,
    // AXI parameters
    parameter AXI_DATA_WIDTH = 32,
    parameter AXI_ADDR_WIDTH = 8
)(
    //----------------------------------------------------
    // AXI4-Lite Slave Interface (bus_clk domain)
    //----------------------------------------------------
    input  wire                       S_AXI_ACLK,
    input  wire                       S_AXI_ARESETN,   // active-low

    // Write address channel
    input  wire [AXI_ADDR_WIDTH-1:0]  S_AXI_AWADDR,
    input  wire [2:0]                 S_AXI_AWPROT,    // ignored (tie to 0)
    input  wire                       S_AXI_AWVALID,
    output reg                        S_AXI_AWREADY,

    // Write data channel
    input  wire [AXI_DATA_WIDTH-1:0]  S_AXI_WDATA,
    input  wire [AXI_DATA_WIDTH/8-1:0] S_AXI_WSTRB,   // byte enables
    input  wire                       S_AXI_WVALID,
    output reg                        S_AXI_WREADY,

    // Write response channel
    output reg  [1:0]                 S_AXI_BRESP,     // 2'b00 = OKAY
    output reg                        S_AXI_BVALID,
    input  wire                       S_AXI_BREADY,

    // Read address channel
    input  wire [AXI_ADDR_WIDTH-1:0]  S_AXI_ARADDR,
    input  wire [2:0]                 S_AXI_ARPROT,    // ignored
    input  wire                       S_AXI_ARVALID,
    output reg                        S_AXI_ARREADY,

    // Read data channel
    output reg  [AXI_DATA_WIDTH-1:0]  S_AXI_RDATA,
    output reg  [1:0]                 S_AXI_RRESP,     // 2'b00 = OKAY
    output reg                        S_AXI_RVALID,
    input  wire                       S_AXI_RREADY,

    //----------------------------------------------------
    // MAC core clock domain
    //----------------------------------------------------
    input  wire                       mac_clk,
    input  wire                       mac_rst,         // active-high

    //----------------------------------------------------
    // Debug
    //----------------------------------------------------
    output wire                       done_led
);

    //----------------------------------------------------
    // Internal wires to mac_accel core
    //----------------------------------------------------
    wire                       bus_clk  = S_AXI_ACLK;
    wire                       bus_rst  = ~S_AXI_ARESETN;  // convert to active-high

    reg                        core_we;
    reg                        core_re;
    reg  [3:0]                 core_addr;
    reg  [AXI_DATA_WIDTH-1:0]  core_wdata;
    wire [AXI_DATA_WIDTH-1:0]  core_rdata;

    //----------------------------------------------------
    // mac_accel core instantiation
    //----------------------------------------------------
    mac_accel #(
        .DATA_WIDTH(DATA_WIDTH),
        .VEC_LEN   (VEC_LEN)
    ) u_mac_accel (
        .bus_clk  (bus_clk),
        .bus_rst  (bus_rst),
        .mac_clk  (mac_clk),
        .mac_rst  (mac_rst),
        .we       (core_we),
        .re       (core_re),
        .addr     (core_addr),
        .wdata    (core_wdata),
        .rdata    (core_rdata),
        .done_led (done_led)
    );

    //----------------------------------------------------
    // Address decode (byte address → 4-bit word index)
    // Offset: 0x00→0, 0x04→1, 0x08→2, 0x0C→3, 0x10→4, 0x14→5
    //----------------------------------------------------
    wire [3:0] wr_word_addr = S_AXI_AWADDR[5:2];
    wire [3:0] rd_word_addr = S_AXI_ARADDR[5:2];

    //----------------------------------------------------
    // Write FSM
    // AXI4-Lite allows AWVALID and WVALID to arrive independently.
    // Latch both channels separately, issue write to core when both are held.
    //----------------------------------------------------
    reg                       aw_latched;
    reg [3:0]                 aw_addr_r;
    reg                       w_latched;
    reg [AXI_DATA_WIDTH-1:0]  w_data_r;
    reg [AXI_DATA_WIDTH/8-1:0] w_strb_r;

    always @(posedge bus_clk) begin
        if (bus_rst) begin
            S_AXI_AWREADY <= 1'b0;
            S_AXI_WREADY  <= 1'b0;
            S_AXI_BVALID  <= 1'b0;
            S_AXI_BRESP   <= 2'b00;
            aw_latched    <= 1'b0;
            w_latched     <= 1'b0;
            aw_addr_r     <= 4'd0;
            w_data_r      <= {AXI_DATA_WIDTH{1'b0}};
            w_strb_r      <= {AXI_DATA_WIDTH/8{1'b0}};
            core_we       <= 1'b0;
            core_addr     <= 4'd0;
            core_wdata    <= {AXI_DATA_WIDTH{1'b0}};
        end else begin
            // Default: deassert single-cycle strobes
            core_we      <= 1'b0;
            S_AXI_AWREADY <= 1'b0;
            S_AXI_WREADY  <= 1'b0;

            // Accept write address
            if (S_AXI_AWVALID && !aw_latched) begin
                aw_addr_r     <= wr_word_addr;
                aw_latched    <= 1'b1;
                S_AXI_AWREADY <= 1'b1;
            end

            // Accept write data
            if (S_AXI_WVALID && !w_latched) begin
                w_data_r   <= S_AXI_WDATA;
                w_strb_r   <= S_AXI_WSTRB;
                w_latched  <= 1'b1;
                S_AXI_WREADY <= 1'b1;
            end

            // Both address and data latched → issue write to core
            if (aw_latched && w_latched && !S_AXI_BVALID) begin
                core_we    <= 1'b1;
                core_addr  <= aw_addr_r;
                // Apply byte strobes
                core_wdata <= {
                    w_strb_r[3] ? w_data_r[31:24] : 8'h00,
                    w_strb_r[2] ? w_data_r[23:16] : 8'h00,
                    w_strb_r[1] ? w_data_r[15:8]  : 8'h00,
                    w_strb_r[0] ? w_data_r[7:0]   : 8'h00
                };
                aw_latched   <= 1'b0;
                w_latched    <= 1'b0;
                S_AXI_BVALID <= 1'b1;
                S_AXI_BRESP  <= 2'b00;  // OKAY
            end

            // Clear BVALID when master accepts response
            if (S_AXI_BVALID && S_AXI_BREADY) begin
                S_AXI_BVALID <= 1'b0;
            end
        end
    end

    //----------------------------------------------------
    // Read FSM
    // 1 cycle latency: accept AR → drive RDATA next cycle
    //----------------------------------------------------
    reg rd_pending;
    reg [3:0] ar_addr_r;

    always @(posedge bus_clk) begin
        if (bus_rst) begin
            S_AXI_ARREADY <= 1'b0;
            S_AXI_RVALID  <= 1'b0;
            S_AXI_RRESP   <= 2'b00;
            S_AXI_RDATA   <= {AXI_DATA_WIDTH{1'b0}};
            rd_pending    <= 1'b0;
            ar_addr_r     <= 4'd0;
            core_re       <= 1'b0;
        end else begin
            core_re       <= 1'b0;
            S_AXI_ARREADY <= 1'b0;

            // Accept read address
            if (S_AXI_ARVALID && !rd_pending && !S_AXI_RVALID) begin
                ar_addr_r     <= rd_word_addr;
                rd_pending    <= 1'b1;
                S_AXI_ARREADY <= 1'b1;
            end

            // One cycle later: drive read data
            if (rd_pending) begin
                core_re      <= 1'b1;
                core_addr    <= ar_addr_r;  // combinatorially read from core
                S_AXI_RDATA  <= core_rdata;
                S_AXI_RRESP  <= 2'b00;
                S_AXI_RVALID <= 1'b1;
                rd_pending   <= 1'b0;
            end

            // Clear RVALID when master accepts data
            if (S_AXI_RVALID && S_AXI_RREADY) begin
                S_AXI_RVALID <= 1'b0;
            end
        end
    end

`ifndef SYNTHESIS
    initial begin
        $display("[MAC_ACCEL_AXI] AXI4-Lite wrapper instantiated, time=%0t", $time);
    end
`endif

endmodule