`default_nettype none
`timescale 1ns/1ps

// ============================================================================
// UDP Engine 100G IP
// ============================================================================
//
// Authors:          M.Subhi Abordan (msubhi_a@mit.edu)
//                   Mena Filfil     (menaf@mit.edu)
// Last Modified:    Dec 3, 2025
//
// ============================================================================
// END
// ============================================================================


module udp_engine_100g #(
    parameter int DATA_WIDTH                    = 512,
    parameter int KEEP_WIDTH                    = DATA_WIDTH/8,

    parameter int IP_UDP_DSCP                   = 0,
    parameter int IP_UDP_ENC                    = 0,
    parameter int IP_UDP_IDEN                   = 0,
    parameter int IP_UDP_FLAGS                  = 0,
    parameter int IP_UDP_FRAG_OFFSET            = 0,
    parameter int IP_UDP_TTL                    = 64,

    parameter int WAYS                          = 4,
    parameter int HASH_WIDTH                    = 16,
    parameter int CONN_ID_WIDTH                 = HASH_WIDTH + $clog2(WAYS),
    parameter int CONNECTION_MANAGER_LATENCY    = 5,

    parameter integer C_S_AXI_DATA_WIDTH	    = 32,
	parameter integer C_S_AXI_ADDR_WIDTH	    = 7,

    localparam int IP_ADDR_WIDTH                = 32,
    localparam int MAC_ADDR_WIDTH               = 48,
    localparam int UDP_PORT_WIDTH               = 16
) (
    // ----------------------------------------------------------------
    // CLOCKS AND RESETS
    // ----------------------------------------------------------------
    input  wire                                 rx_axis_aclk,
    input  wire                                 rx_axis_aresetn,
    input  wire                                 tx_axis_aclk,
    input  wire                                 tx_axis_aresetn,
    input  wire  							    s_axi_aclk,
	input  wire  							    s_axi_aresetn,
    // ----------------------------------------------------------------
    // CMAC INTERFACE (AXI-STREAM)
    // ----------------------------------------------------------------

    // RX Channel
//    input  wire [DATA_WIDTH-1:0]                cmac_rx_axis_tdata,
//    input  wire [KEEP_WIDTH-1:0]                cmac_rx_axis_tkeep,
//    input  wire                                 cmac_rx_axis_tvalid,
//    input  wire                                 cmac_rx_axis_tlast,
//    output logic                                cmac_rx_axis_tready,

    // TX Channel
    output logic [DATA_WIDTH-1:0]               cmac_tx_axis_tdata,
    output logic [KEEP_WIDTH-1:0]               cmac_tx_axis_tkeep,
    output logic                                cmac_tx_axis_tvalid,
    output logic                                cmac_tx_axis_tlast,
    input  wire                                 cmac_tx_axis_tready,

    // ----------------------------------------------------------------
    // UDP INTERFACE (AXI-STREAM)
    // ----------------------------------------------------------------

    // TX Channel
//    input wire  [CONN_ID_WIDTH-1:0]             udp_tx_axis_connection_id,
    input  wire [DATA_WIDTH-1:0]                udp_tx_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]                udp_tx_axis_tkeep,
    input  wire                                 udp_tx_axis_tvalid,
    input  wire                                 udp_tx_axis_tlast,
    output logic                                udp_tx_axis_tready,

    // RX Channel
//    output logic [CONN_ID_WIDTH-1:0]            udp_rx_axis_connection_id,
//    output logic [DATA_WIDTH-1:0]               udp_rx_axis_tdata,
//    output logic [KEEP_WIDTH-1:0]               udp_rx_axis_tkeep,
//    output logic                                udp_rx_axis_tvalid,
//    output logic                                udp_rx_axis_tlast,
//    input  wire                                 udp_rx_axis_tready,

    // ----------------------------------------------------------------
    // CONTROL INTERFACE (AXI-LITE)
    // ----------------------------------------------------------------
	input wire [C_S_AXI_ADDR_WIDTH-1 : 0] 	    s_axi_awaddr,
	input wire [2 : 0] 						    s_axi_awprot,
	input wire  							    s_axi_awvalid,
	output logic  							    s_axi_awready,
	input wire [C_S_AXI_DATA_WIDTH-1 : 0] 	    s_axi_wdata,
	input wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0]   s_axi_wstrb,
	input wire  							    s_axi_wvalid,
	output logic  							    s_axi_wready,
	output logic [1 : 0] 					    s_axi_bresp,
	output logic  							    s_axi_bvalid,
	input wire  							    s_axi_bready,
	input wire [C_S_AXI_ADDR_WIDTH-1 : 0] 	    s_axi_araddr,
	input wire [2 : 0] 						    s_axi_arprot,
	input wire  							    s_axi_arvalid,
	output logic  							    s_axi_arready,
	output logic [C_S_AXI_DATA_WIDTH-1 : 0]     s_axi_rdata,
	output logic [1 : 0] 					    s_axi_rresp,
	output logic 							    s_axi_rvalid,
	input wire  							    s_axi_rready
);
    // -------------------------------------------------------------------------
    // Register Map
    // -------------------------------------------------------------------------
    logic [31:0] csr_udp_engine_100g__ctrl;                         // 0x0

    logic [31:0] csr_udp_engine_100g__myConfig_macAddr_upper;       // 0x0
    logic [31:0] csr_udp_engine_100g__myConfig_macAddr_lower;       // 0x0
    logic [31:0] csr_udp_engine_100g__myConfig_ipAddr;              // 0x0
    logic [31:0] csr_udp_engine_100g__myConfig_udpPort;             // 0x0

    logic [31:0] csr_udp_engine_100g__connManager_wr_macAddr_upper; // 0x0
    logic [31:0] csr_udp_engine_100g__connManager_wr_macAddr_lower; // 0x0
    logic [31:0] csr_udp_engine_100g__connManager_wr_ip_addr;       // 0x0
    logic [31:0] csr_udp_engine_100g__connManager_wr_port;          // 0x0
    logic [31:0] csr_udp_engine_100g__connManager_wr_bind;          // 0x0
    logic [31:0] csr_udp_engine_100g__connManager_wr_trigger;       // 0x0
    logic [31:0] csr_udp_engine_100g__connManager_wr_status;        // 0x0
    logic [31:0] csr_udp_engine_100g__connManager_wr_connectedId;   // 0x0

    // -------------------------------------------------------------------------
    // Register Map FSM 
    // -------------------------------------------------------------------------
 
    udp_engine_control_slave_lite_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH),
		.C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH)
    )
    udp_engine_control_fsm_unit (
        .csr_udp_engine_100g__ctrl(csr_udp_engine_100g__ctrl),
		.csr_udp_engine_100g__myConfig_macAddr_upper(csr_udp_engine_100g__myConfig_macAddr_upper),
		.csr_udp_engine_100g__myConfig_macAddr_lower(csr_udp_engine_100g__myConfig_macAddr_lower),
		.csr_udp_engine_100g__myConfig_ipAddr(csr_udp_engine_100g__myConfig_ipAddr),
		.csr_udp_engine_100g__myConfig_udpPort(csr_udp_engine_100g__myConfig_udpPort),
		.csr_udp_engine_100g__connManager_wr_macAddr_upper(csr_udp_engine_100g__connManager_wr_macAddr_upper),
		.csr_udp_engine_100g__connManager_wr_macAddr_lower(csr_udp_engine_100g__connManager_wr_macAddr_lower),
		.csr_udp_engine_100g__connManager_wr_ip_addr(csr_udp_engine_100g__connManager_wr_ip_addr),
		.csr_udp_engine_100g__connManager_wr_port(csr_udp_engine_100g__connManager_wr_port),
		.csr_udp_engine_100g__connManager_wr_bind(csr_udp_engine_100g__connManager_wr_bind),
		.csr_udp_engine_100g__connManager_wr_trigger(csr_udp_engine_100g__connManager_wr_trigger),
		.csr_udp_engine_100g__connManager_wr_status(csr_udp_engine_100g__connManager_wr_status),
		.csr_udp_engine_100g__connManager_wr_connectedId(csr_udp_engine_100g__connManager_wr_connectedId),

		.S_AXI_ACLK(s_axi_aclk),
		.S_AXI_ARESETN(s_axi_aresetn),
		.S_AXI_AWADDR(s_axi_awaddr),
		.S_AXI_AWPROT(s_axi_awprot),
		.S_AXI_AWVALID(s_axi_awvalid),
		.S_AXI_AWREADY(s_axi_awready),
		.S_AXI_WDATA(s_axi_wdata),
		.S_AXI_WSTRB(s_axi_wstrb),
		.S_AXI_WVALID(s_axi_wvalid),
		.S_AXI_WREADY(s_axi_wready),
		.S_AXI_BRESP(s_axi_bresp),
		.S_AXI_BVALID(s_axi_bvalid),
		.S_AXI_BREADY(s_axi_bready),
		.S_AXI_ARADDR(s_axi_araddr),
		.S_AXI_ARPROT(s_axi_arprot),
		.S_AXI_ARVALID(s_axi_arvalid),
		.S_AXI_ARREADY(s_axi_arready),
		.S_AXI_RDATA(s_axi_rdata),
		.S_AXI_RRESP(s_axi_rresp),
		.S_AXI_RVALID(s_axi_rvalid),
		.S_AXI_RREADY(s_axi_rready)
    );

    // -------------------------------------------------------------------------
    // Registers -> IP Control
    // -------------------------------------------------------------------------

    // IP control registers
    logic tx_engine_enable;
    logic rx_engine_enable;
    logic rx_engine_bypass;
    assign tx_engine_enable = csr_udp_engine_100g__ctrl[0];
    assign rx_engine_enable = csr_udp_engine_100g__ctrl[1];
    assign rx_engine_bypass = csr_udp_engine_100g__ctrl[2];

    // -------------------------------------------------------------------------
    // Registers -> My Internet Configurations
    // -------------------------------------------------------------------------

    // Assume only done while the engine is idle so we have simple cdc
    logic [31:0]     tx_config_ipAddr;
    logic [47:0]     tx_config_macAddr;
    logic [15:0]     tx_config_udpPort;

    logic [31:0]     rx_config_ipAddr;
    logic [47:0]     rx_config_macAddr;
    logic [15:0]     rx_config_udpPort;

    xpm_cdc_array_single #(
        .DEST_SYNC_FF(4),
        .INIT_SYNC_FF(0),
        .SIM_ASSERT_CHK(0),
        .SRC_INPUT_REG(0),
        .WIDTH(32 + 48 + 16)
    ) xpm_cdc_array_single_inst_TX (
        .dest_out({ tx_config_ipAddr, 
                    tx_config_macAddr,
                    tx_config_udpPort}),
        .dest_clk(tx_axis_aclk),
        .src_clk(),
        .src_in(
            {csr_udp_engine_100g__myConfig_ipAddr,
            {csr_udp_engine_100g__myConfig_macAddr_upper[15:0], csr_udp_engine_100g__connManager_wr_macAddr_lower},
            csr_udp_engine_100g__myConfig_udpPort[15:0]})
    );

    xpm_cdc_array_single #(
        .DEST_SYNC_FF(4),
        .INIT_SYNC_FF(0),
        .SIM_ASSERT_CHK(0),
        .SRC_INPUT_REG(0),
        .WIDTH(32 + 48 + 16)
    ) xpm_cdc_array_single_inst_RX (
        .dest_out({ rx_config_ipAddr, 
                    rx_config_macAddr,
                    rx_config_udpPort}),
        .dest_clk(rx_axis_aclk),
        .src_clk(),
        .src_in(
            {csr_udp_engine_100g__myConfig_ipAddr,
            {csr_udp_engine_100g__myConfig_macAddr_upper[15:0], csr_udp_engine_100g__connManager_wr_macAddr_lower},
            csr_udp_engine_100g__myConfig_udpPort[15:0]})
    );

    // -------------------------------------------------------------------------
    // Registers -> Connection Manager Write Channel
    // -------------------------------------------------------------------------

    logic                       s02_axis_ctrl_valid;
    logic [MAC_ADDR_WIDTH-1:0]  s02_axis_ctrl_macAddr;
    logic [IP_ADDR_WIDTH-1:0]   s02_axis_ctrl_ipAddr;
    logic [UDP_PORT_WIDTH-1:0]  s02_axis_ctrl_udpPort;
    logic                       s02_axis_ctrl_bind;
    logic                       s02_axis_ctrl_ready;

    logic                       m02_axis_ctrl_ready;
    logic                       m02_axis_ctrl_valid;
    logic                       m02_axis_ctrl_ack;
    logic [CONN_ID_WIDTH-1:0]   m02_axis_ctrl_connectionId;
    logic                       m02_axis_ctrl_full;

    assign m02_axis_ctrl_ready = 1'b1;
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
                s02_axis_ctrl_valid     <= 1'b0;
                s02_axis_ctrl_macAddr   <=  'b0;
                s02_axis_ctrl_ipAddr    <=  'b0;
                s02_axis_ctrl_udpPort   <=  'b0;
                s02_axis_ctrl_bind      <=  'b0;
        end else begin
            if (s02_axis_ctrl_ready & csr_udp_engine_100g__connManager_wr_trigger) begin
                s02_axis_ctrl_valid     <= 1'b1;
                s02_axis_ctrl_macAddr   <= {csr_udp_engine_100g__connManager_wr_macAddr_upper, csr_udp_engine_100g__connManager_wr_macAddr_lower};
                s02_axis_ctrl_ipAddr    <=  csr_udp_engine_100g__connManager_wr_ip_addr;
                s02_axis_ctrl_udpPort   <=  csr_udp_engine_100g__connManager_wr_port;
                s02_axis_ctrl_bind      <=  csr_udp_engine_100g__connManager_wr_bind;
            end else begin
                s02_axis_ctrl_valid     <= 1'b0;
                s02_axis_ctrl_macAddr   <=  'b0;
                s02_axis_ctrl_ipAddr    <=  'b0;
                s02_axis_ctrl_udpPort   <=  'b0;
                s02_axis_ctrl_bind      <=  'b0;
            end
        end

        if (!s_axi_aresetn) begin
            csr_udp_engine_100g__connManager_wr_status              <= 'b0;
            csr_udp_engine_100g__connManager_wr_connectedId         <= 'b0;
        end else begin
            if (m02_axis_ctrl_valid & m02_axis_ctrl_ready) begin
                csr_udp_engine_100g__connManager_wr_connectedId[CONN_ID_WIDTH-1:0]  <= m02_axis_ctrl_connectionId;
                csr_udp_engine_100g__connManager_wr_status[0]                       <= m02_axis_ctrl_ack;
                csr_udp_engine_100g__connManager_wr_status[1]                       <= m02_axis_ctrl_full;
                csr_udp_engine_100g__connManager_wr_status[31:2]                    <= 30'b0;
            end else begin
                if (s02_axis_ctrl_ready & csr_udp_engine_100g__connManager_wr_trigger) begin
                    csr_udp_engine_100g__connManager_wr_status          <= 32'b0;
                    csr_udp_engine_100g__connManager_wr_connectedId     <= 32'b0;
                end
            end
        end
    end


    // -------------------------------------------------------------------------
    // Connection Manager
    // -------------------------------------------------------------------------

    logic                       s01_axis_rv_lookup_valid;
    logic [CONN_ID_WIDTH-1:0]   s01_axis_rv_lookup_connectionId;
    logic                       s01_axis_rv_lookup_ready;

    logic                       m01_axis_rv_lookup_ready;
    logic                       m01_axis_rv_lookup_valid;
    logic                       m01_axis_rv_lookup_hit;
    logic [MAC_ADDR_WIDTH-1:0]    m01_axis_rv_lookup_macAddr;
    logic [IP_ADDR_WIDTH-1:0]     m01_axis_rv_lookup_ipAddr;
    logic [UDP_PORT_WIDTH-1:0]    m01_axis_rv_lookup_udpPort;

    logic                       s00_axis_fw_lookup_aclk;
    logic                       s00_axis_fw_lookup_aresetn;
    logic                       s00_axis_fw_lookup_valid;
    logic [IP_ADDR_WIDTH-1:0]   s00_axis_fw_lookup_ipAddr;
    logic                       s00_axis_fw_lookup_ready;

    logic                       m00_axis_fw_lookup_ready;
    logic                       m00_axis_fw_lookup_valid;
    logic                       m00_axis_fw_lookup_hit;
    logic [CONN_ID_WIDTH-1:0]   m00_axis_fw_lookup_connectionId;

    connection_manager #(
        .WAYS(WAYS),
        .BRAM_LATENCY(CONNECTION_MANAGER_LATENCY)
    ) connection_manager_unit (
        // Forward Lookup Channel
        .s00_axis_fw_lookup_aclk(rx_axis_aclk),
        .s00_axis_fw_lookup_aresetn(rx_axis_aresetn),
        .s00_axis_fw_lookup_valid(s00_axis_fw_lookup_valid),
        .s00_axis_fw_lookup_ipAddr(s00_axis_fw_lookup_ipAddr),
        .s00_axis_fw_lookup_ready(s00_axis_fw_lookup_ready),

        .m00_axis_fw_lookup_ready(m00_axis_fw_lookup_ready),
        .m00_axis_fw_lookup_valid(m00_axis_fw_lookup_valid),
        .m00_axis_fw_lookup_hit(m00_axis_fw_lookup_hit),
        .m00_axis_fw_lookup_connectionId(m00_axis_fw_lookup_connectionId),

        // Reverse Lookup Channel
        .s01_axis_rv_lookup_aclk(tx_axis_aclk),
        .s01_axis_rv_lookup_aresetn(tx_axis_aresetn),
        .s01_axis_rv_lookup_valid(s01_axis_rv_lookup_valid),
        .s01_axis_rv_lookup_connectionId(s01_axis_rv_lookup_connectionId),
        .s01_axis_rv_lookup_ready(s01_axis_rv_lookup_ready),

        .m01_axis_rv_lookup_ready(m01_axis_rv_lookup_ready),
        .m01_axis_rv_lookup_valid(m01_axis_rv_lookup_valid),
        .m01_axis_rv_lookup_hit(m01_axis_rv_lookup_hit),
        .m01_axis_rv_lookup_macAddr(m01_axis_rv_lookup_macAddr),
        .m01_axis_rv_lookup_ipAddr(m01_axis_rv_lookup_ipAddr),
        .m01_axis_rv_lookup_udpPort(m01_axis_rv_lookup_udpPort),

        // Control (Writes) Channel
        .s02_axis_ctrl_aclk(s_axi_aclk),
        .s02_axis_ctrl_aresetn(s_axi_aresetn),
        .s02_axis_ctrl_valid(s02_axis_ctrl_valid),
        .s02_axis_ctrl_macAddr(s02_axis_ctrl_macAddr),
        .s02_axis_ctrl_ipAddr(s02_axis_ctrl_ipAddr),
        .s02_axis_ctrl_udpPort(s02_axis_ctrl_udpPort),
        .s02_axis_ctrl_bind(s02_axis_ctrl_bind),
        .s02_axis_ctrl_ready(s02_axis_ctrl_ready),

        .m02_axis_ctrl_ready(m02_axis_ctrl_ready),
        .m02_axis_ctrl_valid(m02_axis_ctrl_valid),
        .m02_axis_ctrl_ack(m02_axis_ctrl_ack),
        .m02_axis_ctrl_connectionId(m02_axis_ctrl_connectionId),
        .m02_axis_ctrl_full(m02_axis_ctrl_full)
    );


    // -------------------------------------------------------------------------
    // UDP TX Engine
    // -------------------------------------------------------------------------

    ethernet_tx #(
        .DATA_WIDTH(DATA_WIDTH),
        .CONN_ID_WIDTH(CONN_ID_WIDTH),
        .IP_UDP_DSCP(IP_UDP_DSCP),
        .IP_UDP_ENC(IP_UDP_ENC),
        .IP_UDP_IDEN(IP_UDP_IDEN),
        .IP_UDP_FLAGS(IP_UDP_FLAGS),
        .IP_UDP_FRAG_OFFSET(IP_UDP_FRAG_OFFSET),
        .IP_UDP_TTL(IP_UDP_TTL)
    ) ethernet_tx_unit (
        .tx_axis_aclk(tx_axis_aclk),
        .tx_axis_aresetn(tx_axis_aresetn),
        .tx_engine_enable(tx_engine_enable),

        .my_config_ipAddr(tx_config_ipAddr),
        .my_config_macAddr(tx_config_macAddr),
        .my_config_udpPort(tx_config_udpPort),

        .cmac_tx_axis_tready(cmac_tx_axis_tready),
        .cmac_tx_axis_tdata(cmac_tx_axis_tdata),
        .cmac_tx_axis_tkeep(cmac_tx_axis_tkeep),
        .cmac_tx_axis_tvalid(cmac_tx_axis_tvalid),
        .cmac_tx_axis_tlast(cmac_tx_axis_tlast),

        .udp_tx_axis_tready(udp_tx_axis_tready),
        .udp_tx_axis_tdata(udp_tx_axis_tdata),
        .udp_tx_axis_tkeep(udp_tx_axis_tkeep),
        .udp_tx_axis_tvalid(udp_tx_axis_tvalid),
        .udp_tx_axis_tlast(udp_tx_axis_tlast),

        .m01_axis_rv_lookup_valid(m01_axis_rv_lookup_valid),
        .m01_axis_rv_lookup_connectionId(s01_axis_rv_lookup_connectionId),
        .m01_axis_rv_lookup_ready(m01_axis_rv_lookup_ready),
        .s01_axis_rv_lookup_ready(s01_axis_rv_lookup_ready),
        .s01_axis_rv_lookup_valid(s01_axis_rv_lookup_valid),
        .s01_axis_rv_lookup_hit(m01_axis_rv_lookup_hit),
        .s01_axis_rv_lookup_macAddr(m01_axis_rv_lookup_macAddr),
        .s01_axis_rv_lookup_ipAddr(m01_axis_rv_lookup_ipAddr),
        .s01_axis_rv_lookup_udpPort(m01_axis_rv_lookup_udpPort)
    );

    // -------------------------------------------------------------------------
    // UDP RX Engine (TODO)
    // -------------------------------------------------------------------------


endmodule
`default_nettype wire







