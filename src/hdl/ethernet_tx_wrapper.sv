`default_nettype none
`timescale 1ns/1ps

`include "udp_engine_100g.svh"

// wapper around connection manager and tx

module ethernet_tx_wrapper #(
    parameter int DATA_WIDTH                    = 512,
    parameter int WAYS                          = 4,
    parameter int BRAM_LATENCY                  = 5,
    parameter int CONN_ID_WIDTH                 = HASH_WIDTH + $clog2(WAYS)
) (
    input wire [MAC_ADDR_WIDTH-1:0]             my_config_dst_macAddr,
    input wire [MAC_ADDR_WIDTH-1:0]             my_config_src_macAddr,
    input wire [IP_ADDR_WIDTH-1:0]              my_config_src_ipAddr,
    input wire [UDP_PORT_WIDTH-1:0]             my_config_src_udpPort,

    // Ports of Axi Slave Bus Interface S00_AXIS
    input wire          tx_engine_enable,
    input wire          s00_axis_aclk, 
    input wire          s00_axis_aresetn,
    input wire          s00_axis_tvalid,
    input wire          s00_axis_tlast, 
    input wire [511:0]  s00_axis_tdata,
    input wire [63:0]   s00_axis_tstrb,
    output wire         s00_axis_tready,

    input wire          m00_axis_aclk, 
    input wire          m00_axis_aresetn,
    input wire          m00_axis_tready,
    output wire         m00_axis_tvalid, 
    output wire         m00_axis_tlast,
    output wire [511:0] m00_axis_tdata,
    output wire [63:0]  m00_axis_tstrb,

    // Ports of Axi Slave Bus Interface S02_AXIS
    input wire          s02_axis_aclk, 
    input wire          s02_axis_aresetn,
    input wire          s02_axis_tvalid,
    input wire          s02_axis_tlast, 
    input wire [63:0]   s02_axis_tdata,
    input wire [5:0]    s02_axis_tstrb,
    output wire         s02_axis_tready,

    input wire          m02_axis_aclk, 
    input wire          m02_axis_aresetn,
    input wire          m02_axis_tready,
    output wire         m02_axis_tvalid, 
    output wire         m02_axis_tlast,
    output wire [31:0]  m02_axis_tdata,
    output wire [4:0]   m02_axis_tstrb
);

    logic                           s01_axis_rv_lookup_valid;
    logic [CONN_ID_WIDTH-1:0]       s01_axis_rv_lookup_connectionId;
    logic                           s01_axis_rv_lookup_ready;

    logic                           m01_axis_rv_lookup_ready;
    logic                           m01_axis_rv_lookup_valid;
    logic                           m01_axis_rv_lookup_hit;
    logic [IP_ADDR_WIDTH-1:0]       m01_axis_rv_lookup_ipAddr;
    logic [UDP_PORT_WIDTH-1:0]      m01_axis_rv_lookup_udpPort;


    assign m02_axis_tlast = 1'b1;
    assign m02_axis_tstrb = ~('b0);
    assign m02_axis_tdata[31:CONN_ID_WIDTH+2]   = 'b0;

    connection_manager #(
        .WAYS(WAYS),
        .BRAM_LATENCY(BRAM_LATENCY)
    ) connection_manager_unit (
        // Forward Lookup Channel
        .s00_axis_fw_lookup_aclk(),
        .s00_axis_fw_lookup_aresetn(),
        .s00_axis_fw_lookup_valid(),
        .s00_axis_fw_lookup_ipAddr(),
        .s00_axis_fw_lookup_udpPort(),
        .s00_axis_fw_lookup_ready(),

        .m00_axis_fw_lookup_ready(),
        .m00_axis_fw_lookup_valid(),
        .m00_axis_fw_lookup_hit(),
        .m00_axis_fw_lookup_connectionId(),

        // Reverse Lookup Channel
        .s01_axis_rv_lookup_aclk(s00_axis_aclk),
        .s01_axis_rv_lookup_aresetn(s00_axis_aresetn),
        .s01_axis_rv_lookup_valid(s01_axis_rv_lookup_valid),
        .s01_axis_rv_lookup_connectionId(s01_axis_rv_lookup_connectionId),
        .s01_axis_rv_lookup_ready(s01_axis_rv_lookup_ready),

        .m01_axis_rv_lookup_ready(m01_axis_rv_lookup_ready),
        .m01_axis_rv_lookup_valid(m01_axis_rv_lookup_valid),
        .m01_axis_rv_lookup_hit(m01_axis_rv_lookup_hit),
        .m01_axis_rv_lookup_ipAddr(m01_axis_rv_lookup_ipAddr),
        .m01_axis_rv_lookup_udpPort(m01_axis_rv_lookup_udpPort),

        // Control (Writes) Channel
        .s02_axis_ctrl_aclk(s00_axis_aclk),
        .s02_axis_ctrl_aresetn(s00_axis_aresetn),
        .s02_axis_ctrl_valid(s02_axis_tvalid),
        .s02_axis_ctrl_ipAddr(s02_axis_tdata[31:0]),
        .s02_axis_ctrl_udpPort(s02_axis_tdata[47:32]),
        .s02_axis_ctrl_bind(s02_axis_tdata[48]),
        .s02_axis_ctrl_ready(s02_axis_tready),

        .m02_axis_ctrl_ready(m02_axis_tready),
        .m02_axis_ctrl_valid(m02_axis_tvalid),
        .m02_axis_ctrl_ack(m02_axis_tdata[CONN_ID_WIDTH]),
        .m02_axis_ctrl_full(m02_axis_tdata[CONN_ID_WIDTH+1]),
        .m02_axis_ctrl_connectionId(m02_axis_tdata[CONN_ID_WIDTH-1:0])
    );

    ethernet_tx #(
        .DATA_WIDTH(DATA_WIDTH),
        .CONN_ID_WIDTH(CONN_ID_WIDTH)
    ) ethernet_tx_unit (
        .tx_axis_aclk(s00_axis_aclk),
        .tx_axis_aresetn(s00_axis_aresetn),
        .tx_engine_enable(tx_engine_enable),

        .my_config_dst_macAddr(my_config_dst_macAddr),
        .my_config_src_macAddr(my_config_src_macAddr),
        .my_config_src_ipAddr(my_config_src_ipAddr),
        .my_config_src_udpPort(my_config_src_udpPort),

        .cmac_tx_axis_tready(m00_axis_tready),
        .cmac_tx_axis_tdata(m00_axis_tdata),
        .cmac_tx_axis_tkeep(m00_axis_tstrb),
        .cmac_tx_axis_tvalid(m00_axis_tvalid),
        .cmac_tx_axis_tlast(m00_axis_tlast),

        .udp_tx_axis_tready(s00_axis_tready),
        .udp_tx_axis_tdata(s00_axis_tdata),
        .udp_tx_axis_tkeep(s00_axis_tstrb),
        .udp_tx_axis_tvalid(s00_axis_tvalid),
        .udp_tx_axis_tlast(s00_axis_tlast),

        .m01_axis_rv_lookup_valid(s01_axis_rv_lookup_valid),
        .m01_axis_rv_lookup_connectionId(s01_axis_rv_lookup_connectionId),
        .m01_axis_rv_lookup_ready(s01_axis_rv_lookup_ready),
        .s01_axis_rv_lookup_ready(m01_axis_rv_lookup_ready),
        .s01_axis_rv_lookup_valid(m01_axis_rv_lookup_valid),
        .s01_axis_rv_lookup_hit(m01_axis_rv_lookup_hit),
        .s01_axis_rv_lookup_ipAddr(m01_axis_rv_lookup_ipAddr),
        .s01_axis_rv_lookup_udpPort(m01_axis_rv_lookup_udpPort)
    );

endmodule
`default_nettype wire