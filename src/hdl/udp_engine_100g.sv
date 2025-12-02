`default_nettype none
`timescale 1ns/1ps

// ============================================================================
// Ethernet TX IP
// ============================================================================
//
// Authors:          M.Subhi Abordan (msubhi_a@mit.edu)
//                   Mena Filfil     (menaf@mit.edu)
// Last Modified:    Dec 2, 2025
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
    parameter int CONNECTION_MANAGER_LATENCY    = 3,

    localparam int IP_ADDR_WIDTH                = 32,
    localparam int MAC_ADDR_WIDTH               = 48,
    localparam int UDP_PORT_WIDTH               = 16
) (
    input  wire                         rx_axis_aclk,
    input  wire                         rx_axis_aresetn,
    input  wire                         tx_axis_aclk,
    input  wire                         tx_axis_aresetn,

    // ----------------------------------------------------------------
    // CMAC INTERFACE (AXI-STREAM)
    // ----------------------------------------------------------------

    // RX Channel
    input  wire [DATA_WIDTH-1:0]        cmac_rx_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]        cmac_rx_axis_tkeep,
    input  wire                         cmac_rx_axis_tvalid,
    input  wire                         cmac_rx_axis_tlast,
    output logic                        cmac_rx_axis_tready,

    // TX Channel
    output logic [DATA_WIDTH-1:0]       cmac_tx_axis_tdata,
    output logic [KEEP_WIDTH-1:0]       cmac_tx_axis_tkeep,
    output logic                        cmac_tx_axis_tvalid,
    output logic                        cmac_tx_axis_tlast,
    input  wire                         cmac_tx_axis_tready,

    // ----------------------------------------------------------------
    // UDP INTERFACE (AXI-STREAM)
    // ----------------------------------------------------------------

    // TX Channel
    input wire  [CONN_ID_WIDTH-1:0]     udp_tx_axis_connection_id,
    input  wire [DATA_WIDTH-1:0]        udp_tx_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]        udp_tx_axis_tkeep,
    input  wire                         udp_tx_axis_tvalid,
    input  wire                         udp_tx_axis_tlast,
    output logic                        udp_tx_axis_tready,

    // RX Channel
    output logic [CONN_ID_WIDTH-1:0]    udp_rx_axis_connection_id,
    output logic [DATA_WIDTH-1:0]       udp_rx_axis_tdata,
    output logic [KEEP_WIDTH-1:0]       udp_rx_axis_tkeep,
    output logic                        udp_rx_axis_tvalid,
    output logic                        udp_rx_axis_tlast,
    input  wire                         udp_rx_axis_tready,

    // ----------------------------------------------------------------
    // CONTROL INTERFACE (AXI-LITE)
    // ----------------------------------------------------------------
    input wire      s_axi_ctrl_aclk,
    input wire      s_axi_ctrl_aresetn,

    input wire      s_axi_ctrl_awvalid,
    input wire      s_axi_ctrl_awaddr,
    output logic    s_axi_ctrl_awready,

    input wire      s_axi_ctrl_wvalid,
    input wire      s_axi_ctrl_wdata,
    output logic    s_axi_ctrl_wready,

    input wire      s_axi_ctrl_bready,
    output logic    s_axi_ctrl_bvalid,
    output logic    s_axi_ctrl_bresp,

    input wire      s_axi_ctrl_arvalid,
    input wire      s_axi_ctrl_araddr,
    output logic    s_axi_ctrl_arready,

    input wire      s_axi_ctrl_rready,
    output logic    s_axi_ctrl_rvalid,
    output logic    s_axi_ctrl_rdata,
    output logic    s_axi_ctrl_rresp
);
    // -------------------------------------------------------------------------
    // Register Map
    // -------------------------------------------------------------------------
    logic [31:0] csr_udp_engine;

    logic [31:0] csr_udp_engine_macAddr_upper;
    logic [31:0] csr_udp_engine_macAddr_lower;
    logic [31:0] csr_udp_engine_ipAddr;
    logic [31:0] csr_udp_engine_port;

    logic [31:0] csr_wr_ctrl_macAddr_upper;
    logic [31:0] csr_wr_ctrl_macAddr_lower;
    logic [31:0] csr_wr_ctrl_ip_addr;
    logic [31:0] csr_wr_ctrl_port;
    logic [31:0] csr_wr_ctrl_bind;
    logic [31:0] csr_wr_ctrl_trigger;
    logic [31:0] csr_wr_ctrl_status;
    logic [31:0] csr_wr_ctrl_connectedId;

    // -------------------------------------------------------------------------
    // Register Map FSM (TODO)
    // -------------------------------------------------------------------------
    // add an fsm module to let the user control the registers through axi-lite
 



    // -------------------------------------------------------------------------
    // Registers -> My IP
    // -------------------------------------------------------------------------
    logic [31:0]     my_config_ipAddr;
    logic [47:0]     my_config_macAddr;
    logic [15:0]     my_config_udpPort;

    always_comb begin
        my_config_ipAddr    = udp_engine_ipAddr;
        my_config_macAddr   = {udp_engine_macAddr_upper[15:0], udp_engine_macAddr_lower};
        my_config_udpPort   = udp_engine_port[15:0];
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
    logic [MAC_ADDR_WIDTH:0]    m01_axis_rv_lookup_macAddr;
    logic [IP_ADDR_WIDTH:0]     m01_axis_rv_lookup_ipAddr;
    logic [UDP_PORT_WIDTH:0]    m01_axis_rv_lookup_udpPort;

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
        .s02_axis_ctrl_aclk(tx_axis_aclk),
        .s02_axis_ctrl_aresetn(tx_axis_aresetn),
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
        .my_config_ipAddr(my_config_ipAddr),
        .my_config_macAddr(my_config_macAddr),
        .my_config_udpPort(my_config_udpPort),

        .cmac_tx_axis_tready(cmac_tx_axis_tready),
        .cmac_tx_axis_tdata(cmac_tx_axis_tdata),
        .cmac_tx_axis_tkeep(cmac_tx_axis_tkeep),
        .cmac_tx_axis_tvalid(cmac_tx_axis_tvalid),
        .cmac_tx_axis_tlast(cmac_tx_axis_tlast),

        .udp_tx_axis_connection_id(udp_tx_axis_connection_id),
        .udp_tx_axis_tready(udp_tx_axis_tready),
        .udp_tx_axis_tdata(udp_tx_axis_tdata),
        .udp_tx_axis_tkeep(udp_tx_axis_tkeep),
        .udp_tx_axis_tvalid(udp_tx_axis_tvalid),
        .udp_tx_axis_tlast(udp_tx_axis_tlast),

        .m01_axis_rv_lookup_valid(m01_axis_rv_lookup_valid),
        .m01_axis_rv_lookup_connectionId(m01_axis_rv_lookup_connectionId),
        .m01_axis_rv_lookup_ready(m01_axis_rv_lookup_ready),
        .s01_axis_rv_lookup_ready(s01_axis_rv_lookup_ready),
        .s01_axis_rv_lookup_valid(s01_axis_rv_lookup_valid),
        .s01_axis_rv_lookup_hit(s01_axis_rv_lookup_hit),
        .s01_axis_rv_lookup_macAddr(s01_axis_rv_lookup_macAddr),
        .s01_axis_rv_lookup_ipAddr(s01_axis_rv_lookup_ipAddr),
        .s01_axis_rv_lookup_udpPort(s01_axis_rv_lookup_udpPort)
    );

    // -------------------------------------------------------------------------
    // UDP RX Engine (TODO)
    // -------------------------------------------------------------------------



endmodule
`default_nettype wire







