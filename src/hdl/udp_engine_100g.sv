`default_nettype none
`timescale 1ns/1ps

// ============================================================================
// UDP Engine IP
// ============================================================================
//
// Authors:          M.Subhi Abordan (msubhi_a@mit.edu)
//                   Mena Filfil     (menaf@mit.edu)
// Last Modified:    Nov 30, 2025
//
// ============================================================================
// END
// ============================================================================


module udp_engine_100g #(
    parameter int DATA_WIDTH    = 512,
    parameter int KEEP_WIDTH    = DATA_WIDTH/8
) (
    input  wire                     rx_axis_aclk,
    input  wire                     rx_axis_aresetn,
    input  wire                     tx_axis_aclk,
    input  wire                     tx_axis_aresetn,

    // ----------------------------------------------------------------
    // CMAC INTERFACE (AXI-STREAM)
    // ----------------------------------------------------------------

    // RX Channel
    input  wire [DATA_WIDTH-1:0]    cmac_rx_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]    cmac_rx_axis_tkeep,
    input  wire                     cmac_rx_axis_tvalid,
    input  wire                     cmac_rx_axis_tlast,
    input  wire [           1:0]    cmac_rx_axis_tuser,
    output logic                    cmac_rx_axis_tready,

    // TX Channel
    output logic [DATA_WIDTH-1:0]   cmac_tx_axis_tdata,
    output logic [KEEP_WIDTH-1:0]   cmac_tx_axis_tkeep,
    output logic                    cmac_tx_axis_tvalid,
    output logic                    cmac_tx_axis_tlast,
    output logic [           1:0]   cmac_tx_axis_tuser,
    input  wire                     cmac_tx_axis_tready,

    // ----------------------------------------------------------------
    // UDP INTERFACE (AXI-STREAM)
    // ----------------------------------------------------------------

    // TX Channel
    input  wire [DATA_WIDTH-1:0]    udp_tx_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]    udp_tx_axis_tkeep,
    input  wire                     udp_tx_axis_tvalid,
    input  wire                     udp_tx_axis_tlast,
    input  wire [           1:0]    udp_tx_axis_tuser,
    output logic                    udp_tx_axis_tready,

    // RX Channel
    output logic [DATA_WIDTH-1:0]   udp_rx_axis_tdata,
    output logic [KEEP_WIDTH-1:0]   udp_rx_axis_tkeep,
    output logic                    udp_rx_axis_tvalid,
    output logic                    udp_rx_axis_tlast,
    output logic [           1:0]   udp_rx_axis_tuser,
    input  wire                     udp_rx_axis_tready,

    // ----------------------------------------------------------------
    // CONTROL INTERFACE (AXI-LITE)
    // ----------------------------------------------------------------

    input wire      s_axi_ctrl_aclk,	    // Input	Clock signal. All/outputs of this bus interface are rising edge aligned with this clock.
    input wire      s_axi_ctrl_aresetn,	    // Input	Active-Low synchronous reset signal

    input wire      s_axi_ctrl_awvalid,	    // Input	Write address valid. This signal indicates that the channel is signaling valid write address.
    input wire      s_axi_ctrl_awaddr,	    // Input	Write address. The write address gives the address of the transaction.
    output logic    s_axi_ctrl_awready,     // Output	Write address ready. This signal indicates that the slave is ready to accept an address.

    input wire      s_axi_ctrl_wvalid,	    // Input	Write valid. This signal indicates that valid write data are available.
    input wire      s_axi_ctrl_wdata,	    // Input	Write data.
    output logic    s_axi_ctrl_wready,      // Output	Write ready. This signal indicates that the slave can accept the write data.

    input wire      s_axi_ctrl_bready,	    // Input	Write response ready. This signal indicates that the master can accept a write response.
    output logic    s_axi_ctrl_bvalid,      // Output	Write response valid. This signal indicates that the channel is signaling a valid write response.
    output logic    s_axi_ctrl_bresp,       // Output	Write response. This signal indicate the status of the write transaction.

    input wire      s_axi_ctrl_arvalid,	    // Input	Read address valid. This signal indicates that the channel is signaling valid read address.
    input wire      s_axi_ctrl_araddr,	    // Input	Read address. The read address gives the address of the transaction.
    output logic    s_axi_ctrl_arready,     // Output	Read address ready. This signal indicates that the slave is ready to accept an address.

    input wire      s_axi_ctrl_rready,	    // Input	Read ready. This signal indicates that the master can accept the read data and response information.
    output logic    s_axi_ctrl_rvalid,      // Output	Read valid. This signal indicates that the channel is signaling the required read data.
    output logic    s_axi_ctrl_rdata,       // Output	Read data.
    output logic    s_axi_ctrl_rresp,	    // Output	Read response. This signal indicate the status of the read transfer.
);

    // -------------------------------------------------------------------------
    // Configurations
    // -------------------------------------------------------------------------
    localparam int WAYS                         = 4;
    localparam int HASH_WIDTH                   = 16;
    localparam int CONN_ID_WIDTH                = HASH_WIDTH + $clog2(WAYS);
    localparam int CONNECTION_MANAGER_LATENCY   = 3;


    // -------------------------------------------------------------------------
    // Register Map
    // -------------------------------------------------------------------------
    logic [31:0] csr_udp_engine;

    logic [31:0] csr_wr_ctrl_macAddr_upper;
    logic [31:0] csr_wr_ctrl_macAddr_lower;
    logic [31:0] csr_wr_ctrl_ip_addr;
    logic [31:0] csr_wr_ctrl_port;
    logic [31:0] csr_wr_ctrl_bind;
    logic [31:0] csr_wr_ctrl_trigger;
    logic [31:0] csr_wr_ctrl_status;
    logic [31:0] csr_wr_ctrl_connectedId;
 
    logic [31:0] udp_engine_macAddr_upper
    logic [31:0] udp_engine_macAddr_lower
    logic [31:0] udp_engine_ipAddr
    logic [31:0] udp_engine_port


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

    logic                       s_rv_lookup_valid;
    logic [CONN_ID_WIDTH-1:0]   s_rv_lookup_connectionId;
    logic                       s_rv_lookup_ready;

    logic                       m_rv_lookup_ready;
    logic                       m_rv_lookup_valid;
    logic                       m_rv_lookup_hit;
    logic [47:0]                m_rv_lookup_macAddr;
    logic [31:0]                m_rv_lookup_ipAddr;
    logic [15:0]                m_rv_lookup_udpPort;


    connection_manager #(
        .WAYS(WAYS),
        .BRAM_LATENCY(CONNECTION_MANAGER_LATENCY)
    ) connection_manager_unit (
        // Forward Lookup Channel
        .s00_axis_fw_lookup_aclk(rx_axis_aclk),
        .s00_axis_fw_lookup_aresetn(rx_axis_aresetn),
        .s00_axis_fw_lookup_valid(),
        .s00_axis_fw_lookup_ipAddr(),
        .s00_axis_fw_lookup_ready(),

        .m00_axis_fw_lookup_ready(),
        .m00_axis_fw_lookup_valid(),
        .m00_axis_fw_lookup_hit(),
        .m00_axis_fw_lookup_connectionId(),

        // Reverse Lookup Channel
        .s01_axis_rv_lookup_aclk(tx_axis_aclk),
        .s01_axis_rv_lookup_aresetn(tx_axis_aresetn),
        .s01_axis_rv_lookup_valid(s_rv_lookup_valid),
        .s01_axis_rv_lookup_connectionId(s_rv_lookup_connectionId),
        .s01_axis_rv_lookup_ready(s_rv_lookup_ready),

        .m01_axis_rv_lookup_ready(m_rv_lookup_ready),
        .m01_axis_rv_lookup_valid(m_rv_lookup_valid),
        .m01_axis_rv_lookup_hit(m_rv_lookup_hit),
        .m01_axis_rv_lookup_macAddr(m_rv_lookup_macAddr),
        .m01_axis_rv_lookup_ipAddr(m_rv_lookup_ipAddr),
        .m01_axis_rv_lookup_udpPort(m_rv_lookup_udpPort),

        // Control (Writes) Channel
        .s02_axis_ctrl_aclk(),
        .s02_axis_ctrl_aresetn(),
        .s02_axis_ctrl_valid(),
        .s02_axis_ctrl_macAddr(),
        .s02_axis_ctrl_ipAddr(),
        .s02_axis_ctrl_udpPort(),
        .s02_axis_ctrl_bind(),
        .s02_axis_ctrl_ready(),

        .m02_axis_ctrl_ready(),
        .m02_axis_ctrl_valid(),
        .m02_axis_ctrl_ack(),
        .m02_axis_ctrl_connectionId(),
        .m02_axis_ctrl_full()
    );


    ethernet_tx #(
        .DATA_WIDTH(DATA_WIDTH),
        .CONNECTION_MANAGER_LATENCY(CONNECTION_MANAGER_LATENCY)
    ) ethernet_tx_unit (
        .tx_axis_aclk(tx_axis_aclk),
        .tx_axis_aresetn(tx_axis_aresetn),

        .cmac_tx_axis_tready(cmac_tx_axis_tready),
        .cmac_tx_axis_tdata(cmac_tx_axis_tdata),
        .cmac_tx_axis_tkeep(cmac_tx_axis_tkeep),
        .cmac_tx_axis_tvalid(cmac_tx_axis_tvalid),
        .cmac_tx_axis_tlast(cmac_tx_axis_tlast),
        .cmac_tx_axis_tuser(cmac_tx_axis_tuser),

        .udp_tx_axis_tready(udp_tx_axis_tready),
        .udp_tx_axis_tdata(udp_tx_axis_tdata),
        .udp_tx_axis_tkeep(udp_tx_axis_tkeep),
        .udp_tx_axis_tvalid(udp_tx_axis_tvalid),
        .udp_tx_axis_tlast(udp_tx_axis_tlast),
        .udp_tx_axis_tuser(udp_tx_axis_tuser),

        // connection manager (connectionId) -> (macAddr, ipAddr, udpPort)
        .m01_axis_rv_lookup_valid(s_rv_lookup_valid),
        .m01_axis_rv_lookup_connectionId(s_rv_lookup_connectionId),
        .m01_axis_rv_lookup_ready(s_rv_lookup_ready),
        .s01_axis_rv_lookup_ready(m_rv_lookup_ready),
        .s01_axis_rv_lookup_valid(m_rv_lookup_valid),
        .s01_axis_rv_lookup_hit(m_rv_lookup_hit),
        .s01_axis_rv_lookup_macAddr(m_rv_lookup_macAddr),
        .s01_axis_rv_lookup_ipAddr(m_rv_lookup_ipAddr),
        .s01_axis_rv_lookup_udpPort(m_rv_lookup_udpPort),

        .my_config_ipAddr(),
        .my_config_macAddr(),
        .my_config_udpPort()
    );




    
endmodule