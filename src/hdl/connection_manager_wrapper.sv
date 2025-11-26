`default_nettype none
`timescale 1ns/1ps

module connection_manager_wrapper #(
    parameter int WAYS        = 4
)(
    // Ports of Axi Slave Bus Interface S00_AXIS
    input wire                          s00_axis_aclk, 
    input wire                          s00_axis_aresetn,
    input wire                          s00_axis_tvalid,
    input wire                          s00_axis_tlast, 
    input wire [63:0]                   s00_axis_tdata,
    input wire [7:0]                    s00_axis_tstrb,
    output logic                        s00_axis_tready,

    // Ports of Axi Master Bus Interface M00_AXIS
    input wire                          m00_axis_aclk, 
    input wire                          m00_axis_aresetn,
    input wire                          m00_axis_tready,
    output logic                        m00_axis_tvalid, 
    output logic                        m00_axis_tlast,
    output logic [63:0]                 m00_axis_tdata,
    output logic [7:0]                  m00_axis_tstrb,

    // Ports of Axi Slave Bus Interface S02_AXIS
    input wire                          s02_axis_aclk, 
    input wire                          s02_axis_aresetn,
    input wire                          s02_axis_tvalid,
    input wire                          s02_axis_tlast, 
    input wire [63:0]                   s02_axis_tdata,
    input wire [7:0]                    s02_axis_tstrb,
    output logic                        s02_axis_tready,

    // Ports of Axi Master Bus Interface M02_AXIS
    input wire                          m02_axis_aclk, 
    input wire                          m02_axis_aresetn,
    input wire                          m02_axis_tready,
    output logic                        m02_axis_tvalid, 
    output logic                        m02_axis_tlast,
    output logic [63:0]                 m02_axis_tdata,
    output logic [7:0]                  m02_axis_tstrb

);

    assign m00_axis_tlast = 1'b1;
    assign m02_axis_tlast = 1'b1;
    assign m00_axis_tstrb = 8'hFF;
    assign m02_axis_tstrb = 8'hFF;

    connection_manager #(
        .WAYS(WAYS)
    ) connection_manager_dut (
        .clk(s00_axis_aclk),
        .rst(!s00_axis_aresetn),

        // Forward lookup
        .s00_axis_fw_lookup_valid(s00_axis_tvalid),
        .s00_axis_fw_lookup_key(s00_axis_tdata),
        .s00_axis_fw_lookup_ready(s00_axis_tready),

        .m00_axis_fw_lookup_ready(m00_axis_tready),
        .m00_axis_fw_lookup_valid(m00_axis_tvalid),
        .m00_axis_fw_lookup_hit(m00_axis_tdata[32]),
        .m00_axis_fw_lookup_resp(m00_axis_tdata[31:0]),

        // Control port
        .s02_axis_ctrl_valid(s02_axis_tvalid),
        .s02_axis_ctrl_key(s02_axis_tdata[31:0]),
        .s02_axis_ctrl_activate(s02_axis_tdata[32]),
        .s02_axis_ctrl_ready(s02_axis_tready),

        .m02_axis_ctrl_ready(m02_axis_tready),
        .m02_axis_ctrl_valid(m02_axis_tvalid),
        .m02_axis_ctrl_ack(m02_axis_tdata[0]),
        .m02_axis_ctrl_full(m02_axis_tdata[1])
    );

endmodule
`default_nettype wire