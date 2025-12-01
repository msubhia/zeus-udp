`default_nettype none
`timescale 1ns/1ps


// ============================================================================
// Connection Manager Wrapper for COCOTB
// ============================================================================
//
// Authors:          M.Subhi Abordan (msubhi_a@mit.edu)
//                   Mena Filfil     (menaf@mit.edu)
// Last Modified:    Nov 30, 2025
//
// note: assumes a single domain
//
// ============================================================================
// END
// ============================================================================


module connection_manager_wrapper #(
    parameter int WAYS = 4
)(
    // Ports of Axi Slave Bus Interface S00_AXIS
    input wire          s00_axis_aclk, 
    input wire          s00_axis_aresetn,
    input wire          s00_axis_tvalid,
    input wire          s00_axis_tlast, 
    input wire [31:0]   s00_axis_tdata,
    input wire [4:0]    s00_axis_tstrb,
    output wire         s00_axis_tready,

    // Ports of Axi Master Bus Interface M00_AXIS
    input wire          m00_axis_aclk, 
    input wire          m00_axis_aresetn,
    input wire          m00_axis_tready,
    output wire         m00_axis_tvalid, 
    output wire         m00_axis_tlast,
    output wire [63:0]  m00_axis_tdata,
    output wire [5:0]   m00_axis_tstrb,

    // Ports of Axi Slave Bus Interface S01_AXIS
    input wire          s01_axis_aclk, 
    input wire          s01_axis_aresetn,
    input wire          s01_axis_tvalid,
    input wire          s01_axis_tlast, 
    input wire [31:0]   s01_axis_tdata,
    input wire [4:0]    s01_axis_tstrb,
    output wire         s01_axis_tready,

    // Ports of Axi Master Bus Interface M01_AXIS
    input wire          m01_axis_aclk, 
    input wire          m01_axis_aresetn,
    input wire          m01_axis_tready,
    output wire         m01_axis_tvalid, 
    output wire         m01_axis_tlast,
    output wire [127:0] m01_axis_tdata,
    output wire [6:0]   m01_axis_tstrb,


    // Ports of Axi Slave Bus Interface S02_AXIS
    input wire          s02_axis_aclk, 
    input wire          s02_axis_aresetn,
    input wire          s02_axis_tvalid,
    input wire          s02_axis_tlast, 
    input wire [127:0]  s02_axis_tdata,
    input wire [6:0]    s02_axis_tstrb,
    output wire         s02_axis_tready,

    // Ports of Axi Master Bus Interface M02_AXIS
    input wire          m02_axis_aclk, 
    input wire          m02_axis_aresetn,
    input wire          m02_axis_tready,
    output wire         m02_axis_tvalid, 
    output wire         m02_axis_tlast,
    output wire [31:0]  m02_axis_tdata,
    output wire [4:0]   m02_axis_tstrb
);

    localparam CONN_ID_WIDTH = 16+$clog2(WAYS);

    assign m00_axis_tlast = 1'b1;
    assign m01_axis_tlast = 1'b1;
    assign m02_axis_tlast = 1'b1;
    assign m00_axis_tstrb = ~('b0);
    assign m01_axis_tstrb = ~('b0);
    assign m02_axis_tstrb = ~('b0);

    assign m00_axis_tdata[31:CONN_ID_WIDTH]     = 'b0;
    assign m00_axis_tdata[63:33]                = 'b0;
    assign m01_axis_tdata[127:97]               = 'b0;
    assign m02_axis_tdata[31:CONN_ID_WIDTH+2]   = 'b0;

    connection_manager #(
        .WAYS(WAYS)
    ) connection_manager_dut (
        // -------------------------------------------------------------------------
        // Forward Lookup Channel
        // -------------------------------------------------------------------------
        .s00_axis_fw_lookup_aclk(s00_axis_aclk),
        .s00_axis_fw_lookup_aresetn(s00_axis_aresetn),
        .s00_axis_fw_lookup_valid(s00_axis_tvalid),
        .s00_axis_fw_lookup_ipAddr(s00_axis_tdata),
        .s00_axis_fw_lookup_ready(s00_axis_tready),

        .m00_axis_fw_lookup_ready(m00_axis_tready),
        .m00_axis_fw_lookup_valid(m00_axis_tvalid),
        .m00_axis_fw_lookup_hit(m00_axis_tdata[32]),
        .m00_axis_fw_lookup_connectionId(m00_axis_tdata[CONN_ID_WIDTH-1:0]),

        // -------------------------------------------------------------------------
        // Reverse Lookup Channel
        // -------------------------------------------------------------------------
        .s01_axis_rv_lookup_aclk(s00_axis_aclk),
        .s01_axis_rv_lookup_aresetn(s00_axis_aresetn),
        .s01_axis_rv_lookup_valid(s01_axis_tvalid),
        .s01_axis_rv_lookup_connectionId(s01_axis_tdata[CONN_ID_WIDTH-1:0]),
        .s01_axis_rv_lookup_ready(s01_axis_tready),

        .m01_axis_rv_lookup_ready(m01_axis_tready),
        .m01_axis_rv_lookup_valid(m01_axis_tvalid),
        .m01_axis_rv_lookup_hit(m01_axis_tdata[96]),
        .m01_axis_rv_lookup_macAddr(m01_axis_tdata[47:0]),
        .m01_axis_rv_lookup_udpPort(m01_axis_tdata[63:48]),
        .m01_axis_rv_lookup_ipAddr(m01_axis_tdata[95:64]),

        // -------------------------------------------------------------------------
        // Control (Write) Channel
        // -------------------------------------------------------------------------
        .s02_axis_ctrl_aclk(s00_axis_aclk),
        .s02_axis_ctrl_aresetn(s00_axis_aresetn),
        .s02_axis_ctrl_valid(s02_axis_tvalid),
        .s02_axis_ctrl_macAddr(s02_axis_tdata[47:0]),
        .s02_axis_ctrl_ipAddr(s02_axis_tdata[95:64]),
        .s02_axis_ctrl_udpPort(s02_axis_tdata[63:48]),
        .s02_axis_ctrl_bind(s02_axis_tdata[96]),
        .s02_axis_ctrl_ready(s02_axis_tready),

        .m02_axis_ctrl_ready(m02_axis_tready),
        .m02_axis_ctrl_valid(m02_axis_tvalid),
        .m02_axis_ctrl_ack(m02_axis_tdata[CONN_ID_WIDTH]),
        .m02_axis_ctrl_full(m02_axis_tdata[CONN_ID_WIDTH+1]),
        .m02_axis_ctrl_connectionId(m02_axis_tdata[CONN_ID_WIDTH-1:0])
    );

endmodule
`default_nettype wire