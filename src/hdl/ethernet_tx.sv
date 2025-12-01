`default_nettype none
`timescale 1ns/1ps

// ============================================================================
// Ethernet TX IP
// ============================================================================
//
// Authors:          M.Subhi Abordan (msubhi_a@mit.edu)
//                   Mena Filfil     (menaf@mit.edu)
// Last Modified:   Nov 28, 2025
//
// ============================================================================
// END
// ============================================================================

module ethernet_tx #(
    parameter int DATA_WIDTH                    = 512,
    parameter int CONNECTION_MANAGER_LATENCY    = 3,

    localparam int IP_ADDR_WIDTH    = 32,
    localparam int MAC_ADDR_WIDTH   = 48,
    localparam int UDP_PORT_WIDTH   = 16,
    localparam int KEEP_WIDTH       = DATA_WIDTH/8
) (
    input wire                      tx_axis_aclk,
    input wire                      tx_axis_aresetn,
    // ----------------------------------------------------------------
    // CMAC TX
    // ----------------------------------------------------------------
    input wire                      cmac_tx_axis_tready,
    output logic [DATA_WIDTH-1:0]   cmac_tx_axis_tdata,
    output logic [KEEP_WIDTH-1:0]   cmac_tx_axis_tkeep,
    output logic                    cmac_tx_axis_tvalid,
    output logic                    cmac_tx_axis_tlast,
    output logic [1:0]              cmac_tx_axis_tuser,

    // ----------------------------------------------------------------
    // USER INPUT
    // ----------------------------------------------------------------
    output logic                    udp_tx_axis_tready,
    input wire [DATA_WIDTH-1:0]     udp_tx_axis_tdata,
    input wire [KEEP_WIDTH-1:0]     udp_tx_axis_tkeep,
    input wire                      udp_tx_axis_tvalid,
    input wire                      udp_tx_axis_tlast,
    input wire [1:0]                udp_tx_axis_tuser,

    // ----------------------------------------------------------------
    // CONNECTION MANAGER REVERSE LOOKUP
    // ----------------------------------------------------------------

    output logic                        m01_axis_rv_lookup_valid,
    output logic [CONN_ID_WIDTH-1:0]    m01_axis_rv_lookup_connectionId,
    input wire                          m01_axis_rv_lookup_ready,

    output logic                        s01_axis_rv_lookup_ready,
    input wire                          s01_axis_rv_lookup_valid,
    input wire                          s01_axis_rv_lookup_hit,
    input wire [MAC_ADDR_WIDTH-1:0]     s01_axis_rv_lookup_macAddr,
    input wire [IP_ADDR_WIDTH-1:0]      s01_axis_rv_lookup_ipAddr,
    input wire [UDP_PORT_WIDTH-1:0]     s01_axis_rv_lookup_udpPort,

    // ----------------------------------------------------------------
    // CONNECTION CONFIGURATION
    // ----------------------------------------------------------------
    input wire [IP_ADDR_WIDTH-1:0]      my_config_ipAddr,
    input wire [MAC_ADDR_WIDTH-1:0]     my_config_macAddr,
    input wire [UDP_PORT_WIDTH-1:0]     my_config_udpPort
);
    
endmodule