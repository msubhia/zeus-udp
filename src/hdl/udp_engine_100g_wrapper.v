`default_nettype none
`timescale 1ns/1ps

module udp_engine_100g_wrapper #(
    parameter DATA_WIDTH                    = 512,
    parameter KEEP_WIDTH                    = DATA_WIDTH/8,

    parameter IP_UDP_DSCP                   = 0,
    parameter IP_UDP_ENC                    = 0,
    parameter IP_UDP_IDEN                   = 0,
    parameter IP_UDP_FLAGS                  = 0,
    parameter IP_UDP_FRAG_OFFSET            = 0,
    parameter IP_UDP_TTL                    = 64,

    parameter WAYS                          = 4,
    parameter HASH_WIDTH                    = 16,
    parameter CONN_ID_WIDTH                 = HASH_WIDTH + $clog2(WAYS),
    parameter CONNECTION_MANAGER_LATENCY    = 5,

    parameter integer C_S_AXI_DATA_WIDTH	    = 32,
    parameter integer C_S_AXI_ADDR_WIDTH	    = 7,

    localparam IP_ADDR_WIDTH                = 32,
    localparam MAC_ADDR_WIDTH               = 48,
    localparam UDP_PORT_WIDTH               = 16
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

//    // RX Channel
//    input  wire [DATA_WIDTH-1:0]                cmac_rx_axis_tdata,
//    input  wire [KEEP_WIDTH-1:0]                cmac_rx_axis_tkeep,
//    input  wire                                 cmac_rx_axis_tvalid,
//    input  wire                                 cmac_rx_axis_tlast,
//    output wire                                 cmac_rx_axis_tready,

    // TX Channel
    output wire [DATA_WIDTH-1:0]                cmac_tx_axis_tdata,
    output wire [KEEP_WIDTH-1:0]                cmac_tx_axis_tkeep,
    output wire                                 cmac_tx_axis_tvalid,
    output wire                                 cmac_tx_axis_tlast,
    input  wire                                 cmac_tx_axis_tready,

    // ----------------------------------------------------------------
    // UDP INTERFACE (AXI-STREAM)
    // ----------------------------------------------------------------

    // TX Channel
//    input  wire [CONN_ID_WIDTH-1:0]             udp_tx_axis_connection_id,
    input  wire [DATA_WIDTH-1:0]                udp_tx_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]                udp_tx_axis_tkeep,
    input  wire                                 udp_tx_axis_tvalid,
    input  wire                                 udp_tx_axis_tlast,
    output wire                                 udp_tx_axis_tready,

    // RX Channel
//    output wire [CONN_ID_WIDTH-1:0]             udp_rx_axis_connection_id,
//    output wire [DATA_WIDTH-1:0]                udp_rx_axis_tdata,
//    output wire [KEEP_WIDTH-1:0]                udp_rx_axis_tkeep,
//    output wire                                 udp_rx_axis_tvalid,
//    output wire                                 udp_rx_axis_tlast,
//    input  wire                                 udp_rx_axis_tready,

    // ----------------------------------------------------------------
    // CONTROL INTERFACE (AXI-LITE)
    // ----------------------------------------------------------------
	input  wire [C_S_AXI_ADDR_WIDTH-1 : 0] 	    s_axi_awaddr,
	input  wire [2 : 0] 						s_axi_awprot,
	input  wire  								s_axi_awvalid,
	output wire  								s_axi_awready,

	input  wire [C_S_AXI_DATA_WIDTH-1 : 0] 	    s_axi_wdata,
	input  wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0]  s_axi_wstrb,
	input  wire  								s_axi_wvalid,
	output wire  								s_axi_wready,

	output wire [1 : 0] 						s_axi_bresp,
	output wire  								s_axi_bvalid,
	input  wire  								s_axi_bready,

	input  wire [C_S_AXI_ADDR_WIDTH-1 : 0] 	    s_axi_araddr,
	input  wire [2 : 0] 						s_axi_arprot,
	input  wire  								s_axi_arvalid,
	output wire  								s_axi_arready,

	output wire [C_S_AXI_DATA_WIDTH-1 : 0]      s_axi_rdata,
	output wire [1 : 0] 						s_axi_rresp,
	output wire 								s_axi_rvalid,
	input  wire  								s_axi_rready
);

    // =========================================================================
    // UDP ENGINE CORE
    // =========================================================================
    udp_engine_100g #(
        .DATA_WIDTH                 (DATA_WIDTH),
        .KEEP_WIDTH                 (KEEP_WIDTH),

        .IP_UDP_DSCP                (IP_UDP_DSCP),
        .IP_UDP_ENC                 (IP_UDP_ENC),
        .IP_UDP_IDEN                (IP_UDP_IDEN),
        .IP_UDP_FLAGS               (IP_UDP_FLAGS),
        .IP_UDP_FRAG_OFFSET         (IP_UDP_FRAG_OFFSET),
        .IP_UDP_TTL                 (IP_UDP_TTL),

        .WAYS                       (WAYS),
        .HASH_WIDTH                 (HASH_WIDTH),
        .CONN_ID_WIDTH              (CONN_ID_WIDTH),
        .CONNECTION_MANAGER_LATENCY (CONNECTION_MANAGER_LATENCY),

        .C_S_AXI_DATA_WIDTH         (C_S_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH         (C_S_AXI_ADDR_WIDTH)
    ) udp_engine_100g_unit (
        // ----------------------------------------------------------------
        // CLOCKS AND RESETS
        // ----------------------------------------------------------------
        .rx_axis_aclk               (rx_axis_aclk),
        .rx_axis_aresetn            (rx_axis_aresetn),
        .tx_axis_aclk               (tx_axis_aclk),
        .tx_axis_aresetn            (tx_axis_aresetn),
        .s_axi_aclk                 (s_axi_aclk),
        .s_axi_aresetn              (s_axi_aresetn),

        // ----------------------------------------------------------------
        // CMAC INTERFACE (AXI-STREAM)
        // ----------------------------------------------------------------

        // RX Channel
//        .cmac_rx_axis_tdata         (cmac_rx_axis_tdata),
//        .cmac_rx_axis_tkeep         (cmac_rx_axis_tkeep),
//        .cmac_rx_axis_tvalid        (cmac_rx_axis_tvalid),
//        .cmac_rx_axis_tlast         (cmac_rx_axis_tlast),
//        .cmac_rx_axis_tready        (cmac_rx_axis_tready),

        // TX Channel
        .cmac_tx_axis_tdata         (cmac_tx_axis_tdata),
        .cmac_tx_axis_tkeep         (cmac_tx_axis_tkeep),
        .cmac_tx_axis_tvalid        (cmac_tx_axis_tvalid),
        .cmac_tx_axis_tlast         (cmac_tx_axis_tlast),
        .cmac_tx_axis_tready        (cmac_tx_axis_tready),

        // ----------------------------------------------------------------
        // UDP INTERFACE (AXI-STREAM)
        // ----------------------------------------------------------------

        // TX Channel
//        .udp_tx_axis_connection_id  (udp_tx_axis_connection_id),
        .udp_tx_axis_tdata          (udp_tx_axis_tdata),
        .udp_tx_axis_tkeep          (udp_tx_axis_tkeep),
        .udp_tx_axis_tvalid         (udp_tx_axis_tvalid),
        .udp_tx_axis_tlast          (udp_tx_axis_tlast),
        .udp_tx_axis_tready         (udp_tx_axis_tready),

        // RX Channel
//        .udp_rx_axis_connection_id  (udp_rx_axis_connection_id),
//        .udp_rx_axis_tdata          (udp_rx_axis_tdata),
//        .udp_rx_axis_tkeep          (udp_rx_axis_tkeep),
//        .udp_rx_axis_tvalid         (udp_rx_axis_tvalid),
//        .udp_rx_axis_tlast          (udp_rx_axis_tlast),
//        .udp_rx_axis_tready         (udp_rx_axis_tready),

        // ----------------------------------------------------------------
        // CONTROL INTERFACE (AXI-LITE)
        // ----------------------------------------------------------------
        .s_axi_awaddr               (s_axi_awaddr),
        .s_axi_awprot               (s_axi_awprot),
        .s_axi_awvalid              (s_axi_awvalid),
        .s_axi_awready              (s_axi_awready),

        .s_axi_wdata                (s_axi_wdata),
        .s_axi_wstrb                (s_axi_wstrb),
        .s_axi_wvalid               (s_axi_wvalid),
        .s_axi_wready               (s_axi_wready),

        .s_axi_bresp                (s_axi_bresp),
        .s_axi_bvalid               (s_axi_bvalid),
        .s_axi_bready               (s_axi_bready),

        .s_axi_araddr               (s_axi_araddr),
        .s_axi_arprot               (s_axi_arprot),
        .s_axi_arvalid              (s_axi_arvalid),
        .s_axi_arready              (s_axi_arready),

        .s_axi_rdata                (s_axi_rdata),
        .s_axi_rresp                (s_axi_rresp),
        .s_axi_rvalid               (s_axi_rvalid),
        .s_axi_rready               (s_axi_rready)
    );

endmodule

`default_nettype wire