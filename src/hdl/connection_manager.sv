`default_nettype none
`timescale 1ns/1ps

// ============================================================================
// Connection Manager IP
// ============================================================================
//
// Author:          M.Subhi Abordan (msubhi_a@mit.edu)
// Last Modified:   Nov 28, 2025
//
//
// Description:
//   Hardware block for managing Ethernet/UDP connection metadata using a
//   set-associative hash table. Supports fast forward (IP → connId) and
//   reverse (connId → metadata) lookups, with a control channel for
//   bind/unbind operations.
//
//   The IP is designed to be fully pipelined on lookups, with relaxed
//   consistency between reads and writes. The write path is serialized via an
//   internal state machine.
//
// ----------------------------------------------------------------------------
// Table Organization:
//   - Hash table indexed by HASH_WIDTH-bit hash of the IP address.
//   - Each index contains WAYS associative ways (similar to a cache).
//   - Each way holds: MAC address, IP address, UDP port, valid bit.
//   - Tag match is done on full IP address.
//
//   Hash Function Used: hash[15:0] = ipAddr[31:16] ^ ipAddr[15:0]
//   Connection ID Layout:
//       connId = { way_index , hash_value }
//   where:
//       hash_value : HASH_WIDTH bits
//       way_index  : log2(WAYS) bits
//
// ----------------------------------------------------------------------------
// Lookup Behavior:
//   • Forward Lookup  (IP → connectionId)
//       - Fully pipelined AXI-Stream interface.
//       - Accepts one lookup per cycle (s00_ready is always 1).
//       - Output valid after BRAM_LATENCY cycles.
//       - m00_hit = 1 if a matching IP exists in any way.
//
//   • Reverse Lookup  (connectionId → MAC/IP/Port)
//       - Fully pipelined AXI-Stream interface.
//       - Accepts one request per cycle (s01_ready always 1).
//       - Output valid after BRAM_LATENCY cycles.
//       - Hit only if entry is active.
//
// ----------------------------------------------------------------------------
// Control Port (Bind/Unbind):
//   • Serialized (only one operation in-flight).
//   • s02_ready asserted only when IP can accept a new command.
//   • Bind (activate):
//         - Latency ≈ (BRAM_LATENCY + 1 + WAYS) cycles
//         - Searches all ways at hashed index:
//             * If matching tag exists:  do nothing
//             * Else if free way exists: allocate entry
//             * Else: full → m02_full = 1
//   • Unbind (deactivate):
//         - Latency ≈ (BRAM_LATENCY + 1) cycles
//         - Clears entry indexed by provided IpAddr.
//
//   • If all WAYS for a given hash index are occupied, bind fails.
//   • m02_full = 1, m02_ack = 0.
//   • Software must track and manage active connections to avoid overruns.
//   • Downstream control response port does not apply backpressure to internal
//     state (ack is produced regardless of m02_ready).
//
// ----------------------------------------------------------------------------
// Consistency Model (Relaxed):
//   • Reads do NOT stall or synchronize with writes.
//   • Concurrent read + bind/unbind to same IP may miss or return stale data.
//   • Changes become visible only once the write operation completes.
//
// ============================================================================
// END
// ============================================================================

module connection_manager #(
    // user-manager parameters
    parameter int WAYS              = 4,    // Associativity of hash table
    parameter int BRAM_LATENCY      = 3,    // Latency of underlying BRAM

    // developer-managed parameters
    localparam int HASH_WIDTH       = 16,
    localparam int IP_ADDR_WIDTH    = 32,
    localparam int MAC_ADDR_WIDTH   = 48,
    localparam int UDP_PORT_WIDTH   = 16,
    localparam int TAG_WIDTH        = IP_ADDR_WIDTH,
    localparam int CONN_ID_WIDTH    = HASH_WIDTH+$clog2(WAYS),
    localparam int INDEXES          = 1 << HASH_WIDTH,
    localparam int WAYS_LOG         = $clog2(WAYS)
)(
    input  wire clk,
    input  wire rst, // sync, active high

    // Forward Lookup Channel
    input  wire                         s00_axis_fw_lookup_valid,
    input  wire [IP_ADDR_WIDTH-1:0]     s00_axis_fw_lookup_ipAddr,
    output logic                        s00_axis_fw_lookup_ready,

    input  wire                         m00_axis_fw_lookup_ready, // must be 1
    output logic                        m00_axis_fw_lookup_valid,
    output logic                        m00_axis_fw_lookup_hit,
    output logic [CONN_ID_WIDTH-1:0]    m00_axis_fw_lookup_connectionId,

    // Reverse Lookup Channel
    input  wire                         s01_axis_rv_lookup_valid,
    input  wire [CONN_ID_WIDTH-1:0]     s01_axis_rv_lookup_connectionId,
    output logic                        s01_axis_rv_lookup_ready,

    input  wire                         m01_axis_rv_lookup_ready, // must be 1
    output logic                        m01_axis_rv_lookup_valid,
    output logic                        m01_axis_rv_lookup_hit,
    output logic [MAC_ADDR_WIDTH-1:0]   m01_axis_rv_lookup_macAddr,
    output logic [IP_ADDR_WIDTH-1:0]    m01_axis_rv_lookup_ipAddr,
    output logic [UDP_PORT_WIDTH-1:0]   m01_axis_rv_lookup_udpPort,

    // Control (Writes) Channel
    input  wire                         s02_axis_ctrl_valid,
    input  wire [MAC_ADDR_WIDTH-1:0]    s02_axis_ctrl_macAddr,
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
    // Hash
    // -------------------------------------------------------------------------
    function automatic logic [15:0] xor32to16(input logic [31:0] x);
        return x[31:16] ^ x[15:0];
    endfunction


    // -------------------------------------------------------------------------
    // Per-way BRAM arrays
    // -------------------------------------------------------------------------

    logic [HASH_WIDTH-1:0]      fw_hash_idx;
    logic [TAG_WIDTH-1:0]       fw_tag          [WAYS];
    logic                       fw_valid        [WAYS];

    logic [HASH_WIDTH-1:0]      rv_hash_idx;
    logic [WAYS_LOG-1:0]        rv_way;
    logic [IP_ADDR_WIDTH-1:0]   rv_ipAddr       [WAYS];
    logic [MAC_ADDR_WIDTH-1:0]  rv_macAddr      [WAYS];
    logic [UDP_PORT_WIDTH-1:0]  rv_udpPort      [WAYS];
    logic                       rv_valid        [WAYS];

    logic                       ctrl_wren       [WAYS];
    logic [TAG_WIDTH-1:0]       ctrl_dout_tag   [WAYS];
    logic                       ctrl_dout_valid [WAYS];
    logic [HASH_WIDTH-1:0]      ctrl_addr;
    logic [TAG_WIDTH-1:0]       ctrl_din_tag;
    logic                       ctrl_din_valid;
    logic [IP_ADDR_WIDTH-1:0]   ctrl_din_ipAddr;
    logic [MAC_ADDR_WIDTH-1:0]  ctrl_din_macAddr;
    logic [UDP_PORT_WIDTH-1:0]  ctrl_din_udpPort;

    generate
        for (genvar w=0; w<WAYS; w++) begin : FW_MEMS

            // Tag memory
            dual_port_bram #(
                .DATA_WIDTH(TAG_WIDTH),
                .DATA_DEPTH(INDEXES),
                .BRAM_LATENCY(BRAM_LATENCY)
            ) tag_array (
                .clka(clk), .rsta(rst), .ena(1'b1),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr),
                .dina(ctrl_din_tag),
                .douta(ctrl_dout_tag[w]),

                .clkb(clk), .rstb(rst), .enb(1'b1), .web(1'b0), .dinb('0),
                .addrb(fw_hash_idx),
                .doutb(fw_tag[w])
            );

            // Valid bit memory
            dual_port_bram #(
                .DATA_WIDTH(1),
                .DATA_DEPTH(INDEXES),
                .BRAM_LATENCY(BRAM_LATENCY)
            ) valid_array (
                .clka(clk), .rsta(rst), .ena(1'b1),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr),
                .dina(ctrl_din_valid),
                .douta(ctrl_dout_valid[w]),

                .clkb(clk), .rstb(rst), .enb(1'b1), .web(1'b0), .dinb('0),
                .addrb(fw_hash_idx),
                .doutb(fw_valid[w])
            );

            dual_port_bram #(
                .DATA_WIDTH(MAC_ADDR_WIDTH),
                .DATA_DEPTH(INDEXES),
                .BRAM_LATENCY(BRAM_LATENCY)
            ) macAddr_array (
                .clka(clk), .rsta(rst), .ena(1'b1), .douta(),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr),
                .dina(ctrl_din_macAddr),

                .clkb(clk), .rstb(rst), .enb(1'b1), .web(1'b0), .dinb('0),
                .addrb(rv_hash_idx),
                .doutb(rv_macAddr[w])
            );

            dual_port_bram #(
                .DATA_WIDTH(IP_ADDR_WIDTH),
                .DATA_DEPTH(INDEXES),
                .BRAM_LATENCY(BRAM_LATENCY)
            ) ipAddr_array (
                .clka(clk), .rsta(rst), .ena(1'b1), .douta(),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr),
                .dina(ctrl_din_ipAddr),

                .clkb(clk), .rstb(rst), .enb(1'b1), .web(1'b0), .dinb('0),
                .addrb(rv_hash_idx),
                .doutb(rv_ipAddr[w])
            );

            dual_port_bram #(
                .DATA_WIDTH(UDP_PORT_WIDTH),
                .DATA_DEPTH(INDEXES),
                .BRAM_LATENCY(BRAM_LATENCY)
            ) udpPort_array (
                .clka(clk), .rsta(rst), .ena(1'b1), .douta(),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr),
                .dina(ctrl_din_udpPort),

                .clkb(clk), .rstb(rst), .enb(1'b1), .web(1'b0), .dinb('0),
                .addrb(rv_hash_idx),
                .doutb(rv_udpPort[w])
            );

            dual_port_bram #(
                .DATA_WIDTH(1),
                .DATA_DEPTH(INDEXES),
                .BRAM_LATENCY(BRAM_LATENCY)
            ) valid_array2 (
                .clka(clk), .rsta(rst), .ena(1'b1), .douta(),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr),
                .dina(ctrl_din_valid),

                .clkb(clk), .rstb(rst), .enb(1'b1), .web(1'b0), .dinb('0),
                .addrb(rv_hash_idx),
                .doutb(rv_valid[w])
            );


        end
    endgenerate


    // -------------------------------------------------------------------------
    // Forward Lookup logic
    // -------------------------------------------------------------------------

    // input logic
    assign fw_hash_idx = xor32to16(s00_axis_fw_lookup_ipAddr);

    // Pipeline input for output logic 
    logic [BRAM_LATENCY-1:0][IP_ADDR_WIDTH-1:0]  fw_ipAddr_pipe;
    logic [BRAM_LATENCY-1:0][HASH_WIDTH-1:0]     fw_hash_idx_pipe;
    logic [BRAM_LATENCY-1:0]                     fw_valid_pipe;

    always_ff @(posedge clk) begin
        fw_ipAddr_pipe[0]   <= s00_axis_fw_lookup_ipAddr;
        fw_hash_idx_pipe[0] <= fw_hash_idx;
        fw_valid_pipe[0]    <= s00_axis_fw_lookup_valid;

        for (integer i=1; i<BRAM_LATENCY; i=i+1) begin
            fw_ipAddr_pipe[i]   <= fw_ipAddr_pipe[i-1];
            fw_hash_idx_pipe[i] <= fw_hash_idx_pipe[i-1];
            fw_valid_pipe[i]    <= fw_valid_pipe[i-1];
        end
    end

    // output logic
    always_comb begin
        s00_axis_fw_lookup_ready        = 1'b1;
        m00_axis_fw_lookup_valid        = fw_valid_pipe[BRAM_LATENCY-1];

        m00_axis_fw_lookup_hit          = 1'b0;
        m00_axis_fw_lookup_connectionId = '0;

        for (int w = 0; w < WAYS; w++) begin
            if (fw_valid[w] && fw_tag[w] == fw_ipAddr_pipe[BRAM_LATENCY-1]) begin
                m00_axis_fw_lookup_hit          |= 1'b1;
                m00_axis_fw_lookup_connectionId |= { w[WAYS_LOG-1:0], fw_hash_idx_pipe[BRAM_LATENCY-1] };
            end
        end
    end


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

    always_ff @(posedge clk) begin
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
        m01_axis_rv_lookup_macAddr  = rv_macAddr[rv_way_pipe[BRAM_LATENCY-1]];
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
    logic [MAC_ADDR_WIDTH-1:0]  ctrl_macAddr_q;
    logic [UDP_PORT_WIDTH-1:0]  ctrl_udpPort_q;
    logic [HASH_WIDTH-1:0]      ctrl_hash_idx_q;
    logic                       ctrl_act_q;

    logic [WAYS_LOG-1:0]            way_iter;
    logic [$clog2(BRAM_LATENCY):0]  read_wait;

    assign s02_axis_ctrl_ready = (state == STATE_IDLE);

    assign ctrl_addr = ctrl_hash_idx_q;

    always_ff @(posedge clk) begin
        if (rst) begin
            state                   <= STATE_IDLE;

            ctrl_ipAddr_q           <= 'b0;
            ctrl_macAddr_q          <= 'b0;
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
            ctrl_din_macAddr        <= 'b0;
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
            ctrl_din_macAddr            <= 'b0;
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
                    ctrl_macAddr_q          <= s02_axis_ctrl_macAddr;
                    ctrl_udpPort_q          <= s02_axis_ctrl_udpPort;
                    ctrl_hash_idx_q         <= xor32to16(s02_axis_ctrl_ipAddr);
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
                    ctrl_din_macAddr        <= 'b0;
                    ctrl_din_udpPort        <= 'b0;
                    for (int i=0; i<WAYS; i++) begin
                        ctrl_wren[i] <= (ctrl_dout_tag[i] == ctrl_ipAddr_q);
                    end

                    m02_axis_ctrl_valid     <= 1'b1;
                    m02_axis_ctrl_ack       <= 1'b1;
                    state                   <= STATE_IDLE;
                end

                STATE_ACTIVATE_CHECK: begin
                    logic local_is_any;
                    local_is_any = 1'b0;

                    for (int i=0; i<WAYS; i++) begin
                        if (ctrl_dout_tag[i] == ctrl_ipAddr_q && ctrl_dout_valid[i]) begin
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
                    ctrl_din_tag            <= ctrl_ipAddr_q;
                    ctrl_din_ipAddr         <= ctrl_ipAddr_q;
                    ctrl_din_macAddr        <= ctrl_macAddr_q;
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