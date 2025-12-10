`default_nettype none `timescale 1ns / 1ps

`include "zeus_rpc.svh"

// wapper around connection manager and rx

module ethernet_rx_wrapper #(
    parameter int DATA_WIDTH = 512,
    parameter int KEEP_WIDTH = DATA_WIDTH / 8,

    parameter int IP_UDP_DSCP        = 0,
    parameter int IP_UDP_ENC         = 0,
    parameter int IP_UDP_IDEN        = 0,
    parameter int IP_UDP_FLAGS       = 0,
    parameter int IP_UDP_FRAG_OFFSET = 0,
    parameter int IP_UDP_TTL         = 64,

    parameter int WAYS          = 4,
    parameter int HASH_WIDTH    = 16,
    parameter int CONN_ID_WIDTH = HASH_WIDTH + $clog2(WAYS),

    localparam int IP_ADDR_WIDTH  = 32,
    localparam int MAC_ADDR_WIDTH = 48,
    localparam int UDP_PORT_WIDTH = 16
) (
    input wire        rx_internal_loopback,
    input wire [31:0] my_config_dst_ipAddr,
    input wire [47:0] my_config_dst_macAddr,
    input wire [15:0] my_config_dst_udpPort,
    input wire [47:0] my_config_src_macAddr,

    // Ports of Axi Slave Bus Interface S00_AXIS
    input  wire         s00_axis_aclk,
    input  wire         s00_axis_aresetn,
    input  wire         s00_axis_tvalid,
    input  wire         s00_axis_tlast,
    input  wire [511:0] s00_axis_tdata,
    input  wire [ 63:0] s00_axis_tstrb,
    output wire         s00_axis_tready,

    // input  wire         m00_axis_aclk,
    // input  wire         m00_axis_aresetn,
    input  wire         m00_axis_tready,
    output wire         m00_axis_tvalid,
    output wire         m00_axis_tlast,
    output wire [511:0] m00_axis_tdata,
    output wire [ 63:0] m00_axis_tstrb,

    // Ports of Axi Slave Bus Interface S02_AXIS
    // input  wire         s02_axis_aclk,
    // input  wire         s02_axis_aresetn,
    input  wire         s02_axis_tvalid,
    input  wire         s02_axis_tlast,
    input  wire [127:0] s02_axis_tdata,
    input  wire [  6:0] s02_axis_tstrb,
    output wire         s02_axis_tready,

    // input  wire        m02_axis_aclk,
    // input  wire        m02_axis_aresetn,
    input  wire        m02_axis_tready,
    output wire        m02_axis_tvalid,
    output wire        m02_axis_tlast,
    output wire [31:0] m02_axis_tdata,
    output wire [ 4:0] m02_axis_tstrb
);

  logic                      s01_axis_fw_lookup_valid;
  logic [ CONN_ID_WIDTH-1:0] m01_axis_fw_lookup_connectionId;
  logic                      s01_axis_fw_lookup_ready;

  logic                      m01_axis_fw_lookup_ready;
  logic                      m01_axis_fw_lookup_valid;
  logic                      m01_axis_fw_lookup_hit;
  logic [MAC_ADDR_WIDTH-1:0] m01_axis_fw_lookup_macAddr;
  logic [ IP_ADDR_WIDTH-1:0] s01_axis_fw_lookup_ipAddr;
  logic [UDP_PORT_WIDTH-1:0] s01_axis_fw_lookup_udpPort;


  assign m02_axis_tlast = 1'b1;
  assign m02_axis_tstrb = ~('b0);
  assign m02_axis_tdata[31:CONN_ID_WIDTH+2] = 'b0;

  connection_manager #(
      .WAYS(WAYS)
  ) connection_manager_unit (
      // Forward Lookup Channel
      .s00_axis_fw_lookup_aclk(s00_axis_aclk),
      .s00_axis_fw_lookup_aresetn(s00_axis_aresetn),
      .s00_axis_fw_lookup_valid(s01_axis_fw_lookup_valid),
      .s00_axis_fw_lookup_ipAddr(s01_axis_fw_lookup_ipAddr),
      .s00_axis_fw_lookup_udpPort(s01_axis_fw_lookup_udpPort),
      .s00_axis_fw_lookup_ready(s01_axis_fw_lookup_ready),

      .m00_axis_fw_lookup_ready(m01_axis_fw_lookup_ready),
      .m00_axis_fw_lookup_valid(m01_axis_fw_lookup_valid),
      .m00_axis_fw_lookup_hit(m01_axis_fw_lookup_hit),
      .m00_axis_fw_lookup_connectionId(m01_axis_fw_lookup_connectionId),

      .s01_axis_rv_lookup_aclk(s00_axis_aclk),
      .s01_axis_rv_lookup_aresetn(s00_axis_aresetn),

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

  ethernet_rx #(
      .DATA_WIDTH(DATA_WIDTH),
      .CONN_ID_WIDTH(CONN_ID_WIDTH)
  ) ethernet_rx_unit (
      .rx_axis_aclk(s00_axis_aclk),
      .rx_axis_aresetn(s00_axis_aresetn),
      .rx_internal_loopback(rx_internal_loopback),

      .my_config_dst_ipAddr (my_config_dst_ipAddr),
      .my_config_dst_macAddr(my_config_dst_macAddr),
      .my_config_dst_udpPort(my_config_dst_udpPort),
      .my_config_src_macAddr(my_config_src_macAddr),

      // Slave port s00 from CMAC
      .cmac_rx_axis_tready(s00_axis_tready),
      .cmac_rx_axis_tdata (s00_axis_tdata),
      .cmac_rx_axis_tkeep (s00_axis_tstrb),
      .cmac_rx_axis_tvalid(s00_axis_tvalid),
      .cmac_rx_axis_tlast (s00_axis_tlast),

      // Master port m00 to user logic
      .udp_rx_axis_tready(m00_axis_tready),
      .udp_rx_axis_tdata (m00_axis_tdata),
      .udp_rx_axis_tkeep (m00_axis_tstrb),
      .udp_rx_axis_tvalid(m00_axis_tvalid),
      .udp_rx_axis_tlast (m00_axis_tlast),

      .m01_axis_fw_lookup_valid  (s01_axis_fw_lookup_valid),
      .m01_axis_fw_lookup_ipAddr (s01_axis_fw_lookup_ipAddr),
      .m01_axis_fw_lookup_udpPort(s01_axis_fw_lookup_udpPort),
      .m01_axis_fw_lookup_ready  (s01_axis_fw_lookup_ready),

      .s01_axis_fw_lookup_ready(m01_axis_fw_lookup_ready),
      .s01_axis_fw_lookup_valid(m01_axis_fw_lookup_valid),
      .s01_axis_fw_lookup_hit(m01_axis_fw_lookup_hit),
      .s01_axis_fw_lookup_connectionId(m01_axis_fw_lookup_connectionId)
  );

endmodule
`default_nettype wire
