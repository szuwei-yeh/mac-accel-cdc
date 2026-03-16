`timescale 1ns/1ps

module mac_accel #(
    parameter DATA_WIDTH = 16,
    parameter VEC_LEN    = 4
)(
    // bus clock domain (AXI / MicroBlaze)
    input  wire                     bus_clk,
    input  wire                     bus_rst,

    // MAC core clock domain
    input  wire                     mac_clk,
    input  wire                     mac_rst,

    // Simplified bus interface
    input  wire                     we,
    input  wire                     re,
    input  wire [3:0]               addr,
    input  wire [31:0]              wdata,
    output reg  [31:0]              rdata,

    // debug
    output wire                     done_led   // synchronized done signal
);

    //----------------------------------------
    // Address map
    //----------------------------------------
    localparam ADDR_CTRL     = 4'h0;
    localparam ADDR_AIN      = 4'h1;
    localparam ADDR_BIN      = 4'h2;
    localparam ADDR_MASK     = 4'h3;
    localparam ADDR_RESULT   = 4'h4;
    localparam ADDR_LATENCY  = 4'h5;

    //----------------------------------------
    // bus_clk domain: vecA / vecB / mask
    //----------------------------------------
    reg [DATA_WIDTH-1:0] vecA [0:VEC_LEN-1];
    reg [DATA_WIDTH-1:0] vecB [0:VEC_LEN-1];
    reg [VEC_LEN-1:0]    update_mask;

    reg [3:0] load_idx_a;
    reg [3:0] load_idx_b;

    integer i;

    //----------------------------------------
    // CDC: start pulse (bus -> mac)
    //----------------------------------------
    reg start_toggle_bus;
    reg start_pulse_bus;

    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst) begin
            start_toggle_bus <= 1'b0;
            start_pulse_bus  <= 1'b0;
        end else begin
            start_pulse_bus <= 1'b0;
            if (we && addr == ADDR_CTRL && wdata[0]) begin
                start_toggle_bus <= ~start_toggle_bus;
                start_pulse_bus  <= 1'b1;
            end
        end
    end

    // 2-FF synchronizer: toggle -> mac domain
    reg [1:0] start_sync_mac;
    always @(posedge mac_clk or posedge mac_rst) begin
        if (mac_rst) begin
            start_sync_mac <= 2'b00;
        end else begin
            start_sync_mac <= {start_sync_mac[0], start_toggle_bus};
        end
    end

    // [FIX] Use posedge detection instead of XOR to generate a clean single-cycle pulse.
    // The XOR method can have race conditions when both sync FFs update in the same cycle.
    // Detecting posedge on start_sync_mac[1] ensures exactly one mac_clk cycle pulse.
    reg start_sync_mac_d;
    always @(posedge mac_clk or posedge mac_rst) begin
        if (mac_rst) start_sync_mac_d <= 1'b0;
        else         start_sync_mac_d <= start_sync_mac[1];
    end
    wire start_pulse_mac = start_sync_mac[1] & ~start_sync_mac_d;

    //----------------------------------------
    // Async FIFO: bus_clk -> mac_clk
    //----------------------------------------
    localparam FIFO_WIDTH     = 2*DATA_WIDTH + 1; // {last, a, b}
    localparam FIFO_ADDR_BITS = 4;                // depth=16

    reg                      fifo_wr_en_bus;
    reg  [FIFO_WIDTH-1:0]    fifo_din_bus;
    wire                     fifo_full_bus;

    wire [FIFO_WIDTH-1:0]    fifo_dout_mac;
    wire                     fifo_empty_mac;
    wire                     fifo_rd_en_mac;

    mac_fifo_async #(
        .WIDTH     (FIFO_WIDTH),
        .ADDR_WIDTH(FIFO_ADDR_BITS)
    ) u_fifo (
        .wr_clk (bus_clk),
        .wr_rst (bus_rst),
        .wr_en  (fifo_wr_en_bus),
        .din    (fifo_din_bus),
        .full   (fifo_full_bus),

        .rd_clk (mac_clk),
        .rd_rst (mac_rst),
        .rd_en  (fifo_rd_en_mac),
        .dout   (fifo_dout_mac),
        .empty  (fifo_empty_mac)
    );

    //----------------------------------------
    // bus_clk domain: write vecA/vecB into FIFO
    //----------------------------------------
    reg [3:0] send_idx;
    reg       sending;

    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst) begin
            fifo_wr_en_bus <= 1'b0;
            fifo_din_bus   <= {FIFO_WIDTH{1'b0}};
            send_idx       <= 4'd0;
            sending        <= 1'b0;
        end else begin
            fifo_wr_en_bus <= 1'b0;

            // Start a new vector transfer
            if (start_pulse_bus) begin
                sending  <= 1'b1;
                send_idx <= 4'd0;
            end else if (sending && !fifo_full_bus) begin
                fifo_wr_en_bus <= 1'b1;
                fifo_din_bus   <= {
                    (send_idx == VEC_LEN-1),
                    vecA[send_idx],
                    vecB[send_idx]
                };

                if (send_idx == VEC_LEN-1) begin
                    sending <= 1'b0;
                end else begin
                    send_idx <= send_idx + 1'b1;
                end
            end
        end
    end

    //----------------------------------------
    // bus_clk domain: write registers (vecA/vecB/mask)
    //----------------------------------------
    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst) begin
            update_mask <= {VEC_LEN{1'b0}};
            load_idx_a  <= 4'd0;
            load_idx_b  <= 4'd0;

            for (i = 0; i < VEC_LEN; i = i + 1) begin
                vecA[i] <= {DATA_WIDTH{1'b0}};
                vecB[i] <= {DATA_WIDTH{1'b0}};
            end
        end else begin
            if (we) begin
                case (addr)
                    ADDR_AIN: begin
                        if (load_idx_a < VEC_LEN) begin
                            if (update_mask[load_idx_a])
                                vecA[load_idx_a] <= wdata[DATA_WIDTH-1:0];
                            load_idx_a <= load_idx_a + 1'b1;
                        end
                    end

                    ADDR_BIN: begin
                        if (load_idx_b < VEC_LEN) begin
                            if (update_mask[load_idx_b])
                                vecB[load_idx_b] <= wdata[DATA_WIDTH-1:0];
                            load_idx_b <= load_idx_b + 1'b1;
                        end
                    end

                    ADDR_MASK: begin
                        update_mask <= wdata[VEC_LEN-1:0];
                        load_idx_a  <= 4'd0;
                        load_idx_b  <= 4'd0;
                    end

                    default: ;
                endcase
            end
        end
    end

    //----------------------------------------
    // MAC core (mac_clk domain)
    //----------------------------------------
    wire                          busy_mac;
    wire                          done_mac;
    // [FIX] Use signed type to correctly preserve sign bit from mac_pe result
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

    //----------------------------------------
    // CDC: busy (mac -> bus) - level signal, 2-FF sync is sufficient
    //----------------------------------------
    reg [1:0] busy_sync_bus;

    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst) busy_sync_bus <= 2'b00;
        else         busy_sync_bus <= {busy_sync_bus[0], busy_mac};
    end
    wire busy_bus = busy_sync_bus[1];

    //----------------------------------------
    // CDC: done (mac -> bus)
    // done_mac is a single-cycle pulse; direct 2-FF sync risks missing it.
    // [FIX] Use toggle synchronizer:
    //   mac domain converts done pulse to toggle level
    //   bus domain does 2-FF sync then detects edge to recover the pulse
    //----------------------------------------
    reg done_toggle_mac;
    always @(posedge mac_clk or posedge mac_rst) begin
        if (mac_rst) done_toggle_mac <= 1'b0;
        else if (done_mac) done_toggle_mac <= ~done_toggle_mac;
    end

    reg [1:0] done_sync_bus;
    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst) done_sync_bus <= 2'b00;
        else         done_sync_bus <= {done_sync_bus[0], done_toggle_mac};
    end

    // Edge detection: recover single-cycle pulse in bus domain
    reg done_sync_bus_d;
    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst) done_sync_bus_d <= 1'b0;
        else         done_sync_bus_d <= done_sync_bus[1];
    end
    wire done_pulse_bus = done_sync_bus[1] ^ done_sync_bus_d;

    // done_led: level signal, stays high until next start
    reg done_bus;
    always @(posedge bus_clk or posedge bus_rst) begin
        if (bus_rst)                                      done_bus <= 1'b0;
        else if (we && addr == ADDR_CTRL && wdata[0])     done_bus <= 1'b0; // clear on new start
        else if (done_pulse_bus)                          done_bus <= 1'b1;
    end

    assign done_led = done_bus;

    // Latch result/latency into bus domain registers on done pulse
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

    //----------------------------------------
    // bus_clk domain: register read
    //----------------------------------------
    always @(*) begin
        rdata = 32'h0000_0000;

        case (addr)
            ADDR_CTRL: begin
                rdata = {29'b0, done_bus, busy_bus, 1'b0};
            end

            ADDR_RESULT: begin
                rdata = res_bus_reg;
            end

            ADDR_LATENCY: begin
                rdata = lat_bus_reg;
            end

            default: begin
                rdata = 32'hDEADBEEF;
            end
        endcase
    end

`ifndef SYNTHESIS
    initial begin
        $display("[MAC_ACCEL_CDC] HELLO from %m, time=%0t", $time);
    end
`endif

endmodule