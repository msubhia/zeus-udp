`default_nettype none
`timescale 1ns/1ps

`include "udp_engine_100g.svh"

// ============================================================================
// Connection Manager IP
// ============================================================================
//
// Author:          M.Subhi Abordan (msubhi_a@mit.edu)
//                  Mena Filfil     (menaf@mit.edu)
// Last Modified:   Dec 5, 2025
//
// Description:
//   Hardware block implementing a set-associative lookup structure for mapping:
//       (IP address, UDP port) → connectionId
//   and the reverse:
//       connectionId → (IP address, UDP port)
//
//   The module provides:
//     • Fully pipelined forward lookups (AXIS)
//     • Fully pipelined reverse lookups (AXIS)
//     • Serialized bind / unbind control channel
//
// ----------------------------------------------------------------------------
// Table Organization
// ----------------------------------------------------------------------------
//   Hash index:          HASH_WIDTH-bit hash of {IP, UDP port}
//   Ways per hash index: WAYS (set associativity, similar to a cache)
//
//   Each way contains the following fields:
//       • TAG          : {IP address, UDP port}
//       • IP address   : IP_ADDR_WIDTH bits
//       • UDP port     : UDP_PORT_WIDTH bits
//       • VALID bit    : 1 bit
//
//       connId = { way_index , hash_index }
//
//       hash_index : HASH_WIDTH bits
//       way_index  : log2(WAYS) bits
//
// ----------------------------------------------------------------------------
// Forward Lookup (IP, UDP port → connectionId)
// ----------------------------------------------------------------------------
//   • Fully pipelined AXI-Stream channel
//   • Accepts one lookup per cycle (s00_ready = 1)
//   • Lookup latency = BRAM_LATENCY + 1 cycles
//
//   Match rule:
//       hit if (valid[w] == 1) AND (tag[w] == {ip, udpPort})
//
// ----------------------------------------------------------------------------
// Reverse Lookup (connectionId → IP, UDP port)
// ----------------------------------------------------------------------------
//   • Fully pipelined AXI-Stream channel
//   • Accepts one request per cycle (s01_ready = 1)
//   • Lookup latency = BRAM_LATENCY cycles
//
//   Match rule:
//       hit if valid_bit at requested way is 1
//
// ----------------------------------------------------------------------------
// Control Port (Bind / Unbind)
// ----------------------------------------------------------------------------
//   • Serialized (one command in flight at a time).
//   • s02_ready asserted only when FSM is in STATE_IDLE.
//   • Bind Operation:
//         1. Compute hash index.
//         2. Read all ways at that index.
//         3. If matching tag exists → return existing connId (ack=1, full=0).
//         4. Else if a free way exists (valid=0) → allocate it.
//         5. Else → table full for this index (ack=1, full=1).
//
//   • Unbind Operation:
//         - Clears any entry whose tag matches {IP, UDP port} at this index.
//
//   • Latency (approx):
//         Bind:       BRAM_LATENCY + WAYS + 1 cycles
//         Unbind:     BRAM_LATENCY + 1 cycles
//
//   • Control responses do not apply backpressure to internal state
//       (m02_ready must be 1).
//
// ----------------------------------------------------------------------------
// Consistency Model (Relaxed)
// ----------------------------------------------------------------------------
//   • Forward and reverse lookups do *not* stall for control writes.
//   • Reads may return stale results during bind/unbind sequences.
//   • A new entry becomes visible after the write completes.
//   • Packet pipelines (FW/RV) stay fully throughput-optimal.
//
// ----------------------------------------------------------------------------
// Clocks and Domains
// ----------------------------------------------------------------------------
//   • Forward lookup BRAMs clocked by s00_axis_fw_lookup_aclk
//   • Reverse lookup BRAMs clocked by s01_axis_rv_lookup_aclk
//   • Control write BRAM ports clocked by s02_axis_ctrl_aclk
//
//   All 3 channels operate independently.
//
// ============================================================================
// END
// ============================================================================


module connection_manager #(
    // user-manager parameters
    parameter int WAYS = 4,                     // Associativity of hash table
    parameter int BRAM_LATENCY = 5,             // Latency of underlying RAM

    // developer-managed parameters
    localparam int TAG_WIDTH        = IP_ADDR_WIDTH + UDP_PORT_WIDTH,
    localparam int CONN_ID_WIDTH    = HASH_WIDTH+$clog2(WAYS),
    localparam int INDEXES          = 1 << HASH_WIDTH,
    localparam int WAYS_LOG         = $clog2(WAYS)
)(
    // Forward Lookup Channel
    input  wire                         s00_axis_fw_lookup_aclk,
    input  wire                         s00_axis_fw_lookup_aresetn,
    input  wire                         s00_axis_fw_lookup_valid,
    input  wire [IP_ADDR_WIDTH-1:0]     s00_axis_fw_lookup_ipAddr,
    input  wire [UDP_PORT_WIDTH-1:0]    s00_axis_fw_lookup_udpPort,
    output logic                        s00_axis_fw_lookup_ready,

    input  wire                         m00_axis_fw_lookup_ready, // must be 1
    output logic                        m00_axis_fw_lookup_valid,
    output logic                        m00_axis_fw_lookup_hit,
    output logic [CONN_ID_WIDTH-1:0]    m00_axis_fw_lookup_connectionId,

    // Reverse Lookup Channel
    input  wire                         s01_axis_rv_lookup_aclk,
    input  wire                         s01_axis_rv_lookup_aresetn,
    input  wire                         s01_axis_rv_lookup_valid,
    input  wire [CONN_ID_WIDTH-1:0]     s01_axis_rv_lookup_connectionId,
    output logic                        s01_axis_rv_lookup_ready,

    input  wire                         m01_axis_rv_lookup_ready, // must be 1
    output logic                        m01_axis_rv_lookup_valid,
    output logic                        m01_axis_rv_lookup_hit,
    output logic [IP_ADDR_WIDTH-1:0]    m01_axis_rv_lookup_ipAddr,
    output logic [UDP_PORT_WIDTH-1:0]   m01_axis_rv_lookup_udpPort,

    // Control (Writes) Channel
    input  wire                         s02_axis_ctrl_aclk,
    input  wire                         s02_axis_ctrl_aresetn,
    input  wire                         s02_axis_ctrl_valid,
    input  wire [IP_ADDR_WIDTH-1:0]     s02_axis_ctrl_ipAddr,
    input  wire [UDP_PORT_WIDTH-1:0]    s02_axis_ctrl_udpPort,
    input  wire                         s02_axis_ctrl_bind,
    output logic                        s02_axis_ctrl_ready,

    input  wire                         m02_axis_ctrl_ready, // must be 1
    output logic                        m02_axis_ctrl_valid,
    output logic                        m02_axis_ctrl_ack,
    output logic [CONN_ID_WIDTH-1:0]    m02_axis_ctrl_connectionId,
    output logic                        m02_axis_ctrl_full
);

    // -------------------------------------------------------------------------
    // Signals
    // -------------------------------------------------------------------------

    // RAM <-> FW
    logic [TAG_WIDTH-1:0]       fw_tag          [WAYS];
    logic                       fw_valid        [WAYS];

    // RAM <-> RV
    logic [HASH_WIDTH-1:0]      rv_hash_idx;
    logic [WAYS_LOG-1:0]        rv_way;
    logic [IP_ADDR_WIDTH-1:0]   rv_ipAddr       [WAYS];
    logic [UDP_PORT_WIDTH-1:0]  rv_udpPort      [WAYS];
    logic                       rv_valid        [WAYS];

    // RAM <-> Write Channel FSM
    logic                       ctrl_wren       [WAYS];
    logic [TAG_WIDTH-1:0]       ctrl_dout_tag   [WAYS];
    logic                       ctrl_dout_valid [WAYS];
    logic [HASH_WIDTH-1:0]      ctrl_addr;
    logic [TAG_WIDTH-1:0]       ctrl_din_tag;
    logic                       ctrl_din_valid;
    logic [IP_ADDR_WIDTH-1:0]   ctrl_din_ipAddr;
    logic [UDP_PORT_WIDTH-1:0]  ctrl_din_udpPort;


    // -------------------------------------------------------------------------
    // Forward Lookup logic
    // -------------------------------------------------------------------------

    // Pipeline input for output logic 
    logic [BRAM_LATENCY:0][IP_ADDR_WIDTH-1:0]  fw_ipAddr_pipe;
    logic [BRAM_LATENCY:0][UDP_PORT_WIDTH-1:0] fw_udpPort_pipe;
    logic [BRAM_LATENCY:0][HASH_WIDTH-1:0]     fw_hash_idx_pipe;
    logic [BRAM_LATENCY:0]                     fw_valid_pipe;

    always_ff @(posedge s00_axis_fw_lookup_aclk) begin
        fw_ipAddr_pipe[0]   <= s00_axis_fw_lookup_ipAddr;
        fw_hash_idx_pipe[0] <= hash_fun_ip_port(s00_axis_fw_lookup_ipAddr, s00_axis_fw_lookup_udpPort);
        fw_valid_pipe[0]    <= s00_axis_fw_lookup_valid;
        fw_udpPort_pipe[0]  <= s00_axis_fw_lookup_udpPort;

        for (integer i=1; i<BRAM_LATENCY+1; i=i+1) begin
            fw_ipAddr_pipe[i]   <= fw_ipAddr_pipe[i-1];
            fw_hash_idx_pipe[i] <= fw_hash_idx_pipe[i-1];
            fw_valid_pipe[i]    <= fw_valid_pipe[i-1];
            fw_udpPort_pipe[i]  <= fw_udpPort_pipe[i-1];
        end
    end

    // output logic
    always_comb begin
        s00_axis_fw_lookup_ready        = 1'b1;
        m00_axis_fw_lookup_valid        = fw_valid_pipe[BRAM_LATENCY];

        m00_axis_fw_lookup_hit          = 1'b0;
        m00_axis_fw_lookup_connectionId = '0;

        for (int w = 0; w < WAYS; w++) begin
            if (fw_valid[w] && (fw_tag[w] == {fw_ipAddr_pipe[BRAM_LATENCY], fw_udpPort_pipe[BRAM_LATENCY]})) begin
                m00_axis_fw_lookup_hit          |= 1'b1;
                m00_axis_fw_lookup_connectionId |= { w[WAYS_LOG-1:0], fw_hash_idx_pipe[BRAM_LATENCY] };
            end
        end
    end

    // -------------------------------------------------------------------------
    // Per-way BRAM arrays
    // -------------------------------------------------------------------------

    generate
        for (genvar w=0; w<WAYS; w++) begin : FW_MEMS

            // Tag memory
            dual_port_bram #(
                .DATA_WIDTH(TAG_WIDTH),
                .DATA_DEPTH(INDEXES),
                .BRAM_LATENCY(BRAM_LATENCY)
            ) tag_array (
                .clka(s02_axis_ctrl_aclk), .rsta(!s02_axis_ctrl_aresetn), .ena(1'b1),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr),
                .dina(ctrl_din_tag),
                .douta(ctrl_dout_tag[w]),

                .clkb(s00_axis_fw_lookup_aclk), .rstb(!s00_axis_fw_lookup_aresetn), .enb(1'b1), .web(1'b0), .dinb('0),
                .addrb(fw_hash_idx_pipe[0]),
                .doutb(fw_tag[w])
            );

            // Valid bit memory
            dual_port_bram #(
                .DATA_WIDTH(1),
                .DATA_DEPTH(INDEXES),
                .BRAM_LATENCY(BRAM_LATENCY)
            ) valid_array (
                .clka(s02_axis_ctrl_aclk), .rsta(!s02_axis_ctrl_aresetn), .ena(1'b1),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr),
                .dina(ctrl_din_valid),
                .douta(ctrl_dout_valid[w]),

                .clkb(s00_axis_fw_lookup_aclk), .rstb(!s00_axis_fw_lookup_aresetn), .enb(1'b1), .web(1'b0), .dinb('0),
                .addrb(fw_hash_idx_pipe[0]),
                .doutb(fw_valid[w])
            );

            dual_port_bram #(
                .DATA_WIDTH(IP_ADDR_WIDTH),
                .DATA_DEPTH(INDEXES),
                .BRAM_LATENCY(BRAM_LATENCY)
            ) ipAddr_array (
                .clka(s02_axis_ctrl_aclk), .rsta(!s02_axis_ctrl_aresetn), .ena(1'b1), .douta(),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr),
                .dina(ctrl_din_ipAddr),

                .clkb(s01_axis_rv_lookup_aclk), .rstb(!s01_axis_rv_lookup_aresetn), .enb(1'b1), .web(1'b0), .dinb('0),
                .addrb(rv_hash_idx),
                .doutb(rv_ipAddr[w])
            );

            dual_port_bram #(
                .DATA_WIDTH(UDP_PORT_WIDTH),
                .DATA_DEPTH(INDEXES),
                .BRAM_LATENCY(BRAM_LATENCY)
            ) udpPort_array (
                .clka(s02_axis_ctrl_aclk), .rsta(!s02_axis_ctrl_aresetn), .ena(1'b1), .douta(),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr),
                .dina(ctrl_din_udpPort),

                .clkb(s01_axis_rv_lookup_aclk), .rstb(!s01_axis_rv_lookup_aresetn), .enb(1'b1), .web(1'b0), .dinb('0),
                .addrb(rv_hash_idx),
                .doutb(rv_udpPort[w])
            );

            dual_port_bram #(
                .DATA_WIDTH(1),
                .DATA_DEPTH(INDEXES),
                .BRAM_LATENCY(BRAM_LATENCY)
            ) valid_array2 (
                .clka(s02_axis_ctrl_aclk), .rsta(!s02_axis_ctrl_aresetn), .ena(1'b1), .douta(),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr),
                .dina(ctrl_din_valid),

                .clkb(s01_axis_rv_lookup_aclk), .rstb(!s01_axis_rv_lookup_aresetn), .enb(1'b1), .web(1'b0), .dinb('0),
                .addrb(rv_hash_idx),
                .doutb(rv_valid[w])
            );

        end
    endgenerate


    // -------------------------------------------------------------------------
    // Reverse Lookup logic
    // -------------------------------------------------------------------------

    // input logic
    assign rv_hash_idx  = s01_axis_rv_lookup_connectionId[HASH_WIDTH-1:0];
    assign rv_way       = s01_axis_rv_lookup_connectionId[CONN_ID_WIDTH-1:HASH_WIDTH];

    // Pipeline input for BRAM read latency
    logic [BRAM_LATENCY-1:0][HASH_WIDTH-1:0]    rv_hash_idx_pipe;
    logic [BRAM_LATENCY-1:0][WAYS_LOG-1:0]      rv_way_pipe;
    logic [BRAM_LATENCY-1:0]                    rv_valid_pipe;

    always_ff @(posedge s01_axis_rv_lookup_aclk) begin
        rv_hash_idx_pipe[0] <= rv_hash_idx;
        rv_way_pipe[0]      <= rv_way;
        rv_valid_pipe[0]    <= s01_axis_rv_lookup_valid;

        for (integer i=1; i<BRAM_LATENCY; i=i+1) begin
            rv_hash_idx_pipe[i] <= rv_hash_idx_pipe[i-1];
            rv_way_pipe[i]      <= rv_way_pipe[i-1];
            rv_valid_pipe[i]    <= rv_valid_pipe[i-1];
        end
    end

    // output logic
    always_comb begin
        s01_axis_rv_lookup_ready    = 1'b1;    
        m01_axis_rv_lookup_valid    = rv_valid_pipe[BRAM_LATENCY-1];
        m01_axis_rv_lookup_hit      = rv_valid[rv_way_pipe[BRAM_LATENCY-1]];
        m01_axis_rv_lookup_ipAddr   = rv_ipAddr[rv_way_pipe[BRAM_LATENCY-1]];
        m01_axis_rv_lookup_udpPort  = rv_udpPort[rv_way_pipe[BRAM_LATENCY-1]];
    end



    // -------------------------------------------------------------------------
    // Control (Write) FSM
    // -------------------------------------------------------------------------

    typedef enum logic [2:0] {
        STATE_IDLE,
        STATE_READ,
        STATE_ACTIVATE_CHECK,
        STATE_ACTIVATE,
        STATE_DEACTIVATE
    } state_t;

    state_t state;

    logic [IP_ADDR_WIDTH-1:0]   ctrl_ipAddr_q;
    logic [UDP_PORT_WIDTH-1:0]  ctrl_udpPort_q;
    logic [HASH_WIDTH-1:0]      ctrl_hash_idx_q;
    logic                       ctrl_act_q;

    logic [WAYS_LOG-1:0]            way_iter;
    logic [$clog2(BRAM_LATENCY):0]  read_wait;

    assign s02_axis_ctrl_ready = (state == STATE_IDLE);

    assign ctrl_addr = ctrl_hash_idx_q;

    always_ff @(posedge s02_axis_ctrl_aclk) begin
        if (!s02_axis_ctrl_aresetn) begin
            state                   <= STATE_IDLE;

            ctrl_ipAddr_q           <= 'b0;
            ctrl_udpPort_q          <= 'b0;
            ctrl_hash_idx_q         <= 'b0;
            ctrl_act_q              <= 'b0;

            m02_axis_ctrl_valid     <= 1'b0;
            m02_axis_ctrl_ack       <= 1'b0;
            m02_axis_ctrl_full      <= 1'b0;
            way_iter                <= 'b0;
            read_wait               <= 'b0;

            ctrl_din_tag            <= 1'b0;
            ctrl_din_valid          <= 1'b0;
            ctrl_din_ipAddr         <= 'b0;
            ctrl_din_udpPort        <= 'b0;

            for (int i=0; i<WAYS; i++) begin
                ctrl_wren[i]        <= 1'b0;
            end

            m02_axis_ctrl_connectionId <= 'b0;
        
        end else begin
            // DEFAULTS
            m02_axis_ctrl_connectionId  <= 'b0;
            ctrl_din_tag                <= 1'b0;
            ctrl_din_valid              <= 1'b0;
            ctrl_din_ipAddr             <= 'b0;
            ctrl_din_udpPort            <= 'b0;
            for (int i=0; i<WAYS; i++) begin
                ctrl_wren[i]            <= 1'b0;
            end

            case (state)
                // -------------------------------------------------------------
                STATE_IDLE: begin
                    m02_axis_ctrl_valid     <= 1'b0;
                    m02_axis_ctrl_ack       <= 1'b0;
                    m02_axis_ctrl_full      <= 1'b0;
                    way_iter                <= 'b0;
                    read_wait               <= 'b0;

                    ctrl_ipAddr_q           <= s02_axis_ctrl_ipAddr;
                    ctrl_udpPort_q          <= s02_axis_ctrl_udpPort;
                    ctrl_hash_idx_q         <= hash_fun_ip_port(s02_axis_ctrl_ipAddr, s02_axis_ctrl_udpPort);
                    ctrl_act_q              <= s02_axis_ctrl_bind;

                    state                   <= (s02_axis_ctrl_valid)? STATE_READ: STATE_IDLE;
                end 

                // -------------------------------------------------------------
                STATE_READ: begin
                    read_wait <= read_wait + 1;
                    if (read_wait == BRAM_LATENCY-1) begin
                        state <= ctrl_act_q ? STATE_ACTIVATE_CHECK : STATE_DEACTIVATE;
                    end
                end

                // -------------------------------------------------------------
                STATE_DEACTIVATE: begin
                    ctrl_din_tag            <= 1'b0;
                    ctrl_din_valid          <= 1'b0;
                    ctrl_din_ipAddr         <= 'b0;
                    ctrl_din_udpPort        <= 'b0;
                    for (int i=0; i<WAYS; i++) begin
                        ctrl_wren[i] <= (ctrl_dout_tag[i] == {ctrl_ipAddr_q, ctrl_udpPort_q});
                    end

                    m02_axis_ctrl_valid     <= 1'b1;
                    m02_axis_ctrl_ack       <= 1'b1;
                    state                   <= STATE_IDLE;
                end

                STATE_ACTIVATE_CHECK: begin
                    logic local_is_any;
                    local_is_any = 1'b0;

                    for (int i=0; i<WAYS; i++) begin
                        if ((ctrl_dout_tag[i] == {ctrl_ipAddr_q, ctrl_udpPort_q}) && ctrl_dout_valid[i]) begin
                            m02_axis_ctrl_connectionId[CONN_ID_WIDTH-1:HASH_WIDTH] <= i;
                            local_is_any |= 1;
                        end
                    end

                    if (local_is_any) begin
                        m02_axis_ctrl_valid <= 1'b1;
                        m02_axis_ctrl_ack   <= 1'b1;
                        state               <= STATE_IDLE;
                        m02_axis_ctrl_connectionId[HASH_WIDTH-1:0] <= ctrl_hash_idx_q;
                    end else begin
                        state               <= STATE_ACTIVATE;
                    end
                end

                // -------------------------------------------------------------
                STATE_ACTIVATE: begin
                    ctrl_din_valid          <= 1'b1;
                    ctrl_din_tag            <= {ctrl_ipAddr_q, ctrl_udpPort_q};
                    ctrl_din_ipAddr         <= ctrl_ipAddr_q;
                    ctrl_din_udpPort        <= ctrl_udpPort_q;

                    if (!ctrl_dout_valid[way_iter]) begin
                        ctrl_wren[way_iter]         <= 1'b1;
                        m02_axis_ctrl_valid         <= 1'b1;
                        m02_axis_ctrl_ack           <= 1'b1;
                        m02_axis_ctrl_full          <= 1'b0;
                        state                       <= STATE_IDLE;
                        m02_axis_ctrl_connectionId  <= {way_iter, ctrl_hash_idx_q};

                    end else begin
                        if (way_iter == WAYS-1) begin
                            m02_axis_ctrl_valid     <= 1'b1;
                            m02_axis_ctrl_ack       <= 1'b1;
                            m02_axis_ctrl_full      <= 1'b1;
                            state                   <= STATE_IDLE;
                        end else begin
                            way_iter <= way_iter + 1;
                        end
                    end
                end

                // -------------------------------------------------------------
            endcase

        end
    end

endmodule // connection_manager

`default_nettype wire