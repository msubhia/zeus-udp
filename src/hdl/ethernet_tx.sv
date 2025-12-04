`default_nettype none
`timescale 1ns/1ps

`include "zeus_rpc.svh"

// ============================================================================
// Ethernet TX IP
// ============================================================================
//
// Authors:          M.Subhi Abordan (msubhi_a@mit.edu)
//                   Mena Filfil     (menaf@mit.edu)
// Last Modified:    Dec 3, 2025
//
// ============================================================================
// END
// ============================================================================

function automatic logic [31:0] swap_bytes_4(input logic [31:0] din);
    for (int i = 0; i < 4; i++) begin
        swap_bytes_4[i*8 +: 8] = din[(4-1-i)*8 +: 8];
    end
endfunction

module ethernet_tx #(
    parameter int DATA_WIDTH                    = 512,
    parameter int CONN_ID_WIDTH                 = 18,
    parameter int IP_UDP_DSCP                   = 0,
    parameter int IP_UDP_ENC                    = 0,
    parameter int IP_UDP_IDEN                   = 0,
    parameter int IP_UDP_FLAGS                  = 0,
    parameter int IP_UDP_FRAG_OFFSET            = 0,
    parameter int IP_UDP_TTL                    = 64,

    localparam int IP_ADDR_WIDTH                = 32,
    localparam int MAC_ADDR_WIDTH               = 48,
    localparam int UDP_PORT_WIDTH               = 16,
    localparam int CONNECTION_META_WIDTH        = IP_ADDR_WIDTH + MAC_ADDR_WIDTH + UDP_PORT_WIDTH + 8,
    localparam int IP_PACKET_LENGTH_WIDTH       = 16,
    localparam int ETH_HEADER_BYTES             = 14,
    localparam int IP_HEADER_BYTES              = 20,
    localparam int UDP_HEADER_BYTES             = 8,
    localparam int TOTAL_HEADERS_BYTES          = ETH_HEADER_BYTES + IP_HEADER_BYTES + UDP_HEADER_BYTES,
    localparam int TOTAL_HEADERS_BITS           = TOTAL_HEADERS_BYTES * 8
) (
    // ----------------------------------------------------------------
    // CONTROL AND STATUS
    // ----------------------------------------------------------------
    input wire                          tx_axis_aclk,
    input wire                          tx_axis_aresetn,
    input wire                          tx_engine_enable,

    // ----------------------------------------------------------------
    // CONNECTION CONFIGURATION
    // ----------------------------------------------------------------
    input wire [IP_ADDR_WIDTH-1:0]      my_config_ipAddr,
    input wire [MAC_ADDR_WIDTH-1:0]     my_config_macAddr,
    input wire [UDP_PORT_WIDTH-1:0]     my_config_udpPort,

    // ----------------------------------------------------------------
    // CMAC TX
    // ----------------------------------------------------------------
    input wire                          cmac_tx_axis_tready,
    output logic [DATA_WIDTH-1:0]       cmac_tx_axis_tdata,
    output logic [DATA_WIDTH/8-1:0]     cmac_tx_axis_tkeep,
    output logic                        cmac_tx_axis_tvalid,
    output logic                        cmac_tx_axis_tlast,

    // ----------------------------------------------------------------
    // USER INPUT
    // ----------------------------------------------------------------
    output logic                        udp_tx_axis_tready,
    input wire [DATA_WIDTH-1:0]         udp_tx_axis_tdata,
    input wire [DATA_WIDTH/8-1:0]       udp_tx_axis_tkeep,
    input wire                          udp_tx_axis_tvalid,
    input wire                          udp_tx_axis_tlast,

    // ----------------------------------------------------------------
    // CONNECTION MANAGER REVERSE LOOKUP
    // ----------------------------------------------------------------

    output logic                        m01_axis_rv_lookup_valid,
    output logic [CONN_ID_WIDTH-1:0]    m01_axis_rv_lookup_connectionId,
    input wire                          m01_axis_rv_lookup_ready, // assumed always 1

    output logic                        s01_axis_rv_lookup_ready, // assumed always 1
    input wire                          s01_axis_rv_lookup_valid,
    input wire                          s01_axis_rv_lookup_hit,
    input wire [MAC_ADDR_WIDTH-1:0]     s01_axis_rv_lookup_macAddr,
    input wire [IP_ADDR_WIDTH-1:0]      s01_axis_rv_lookup_ipAddr,
    input wire [UDP_PORT_WIDTH-1:0]     s01_axis_rv_lookup_udpPort
);

    // ----------------------------------------------------------------------------------------------
    // FIFOs
    // ----------------------------------------------------------------------------------------------

    // As packets are entering we count their length on the fly and forward them to a fifo, once an
    // entire packet has entered we send its length in another fifo a long with the fetched connection
    // meta data
    // a consuming fsm on the other side of the fifo starts when all relavent metadata of a packet in
    // the fifo arrives, and it constructs the header and prepend it

    logic ready_packet_fifo;
    logic ready_connection_metadata_fifo;
    logic ready_length_metadata_fifo;

    logic packet_fifo_tready;
    logic packet_fifo_tlast;
    logic packet_fifo_tvalid;
    logic [DATA_WIDTH  -1:0] packet_fifo_tdata;
    logic [DATA_WIDTH/8-1:0] packet_fifo_tkeep;

    logic connection_fifo_tready;
    logic connection_fifo_tlast;
    logic connection_fifo_tvalid;
    logic [CONNECTION_META_WIDTH-1:0] connection_fifo_tdata;

    logic payload_length_fifo_tready;
    logic payload_length_fifo_tlast;
    logic payload_length_fifo_tvalid;
    logic [IP_PACKET_LENGTH_WIDTH-1:0] payload_length_fifo_tdata;

    logic [IP_PACKET_LENGTH_WIDTH-1:0]      current_total_length;
    logic [IP_PACKET_LENGTH_WIDTH-1:0]      current_total_length_post;
    logic                                   current_total_length_post_valid;

    fifo_axis_wrapper #(
        .FIFO_DEPTH(64),
        .TDATA_WIDTH(DATA_WIDTH)
    ) packet_fifo (
        .m_aclk(tx_axis_aclk),
        .m_axis_tready(packet_fifo_tready),
        .m_axis_tlast(packet_fifo_tlast),
        .m_axis_tvalid(packet_fifo_tvalid),
        .m_axis_tdata(packet_fifo_tdata),
        .m_axis_tkeep(packet_fifo_tkeep),

        .s_aclk(tx_axis_aclk),
        .s_aresetn(tx_axis_aresetn),
        .s_axis_tdata(udp_tx_axis_tdata),
        .s_axis_tkeep(udp_tx_axis_tkeep),
        .s_axis_tlast(udp_tx_axis_tlast),
        .s_axis_tvalid(udp_tx_axis_tvalid & tx_engine_enable),
        .s_axis_tready(ready_packet_fifo)
    );

    fifo_axis_wrapper #(
        .FIFO_DEPTH(128),
        .TDATA_WIDTH(CONNECTION_META_WIDTH)
    ) connection_metadata_fifo (
        .m_aclk(tx_axis_aclk),
        .m_axis_tready(connection_fifo_tready),
        .m_axis_tlast(connection_fifo_tlast),
        .m_axis_tvalid(connection_fifo_tvalid),
        .m_axis_tdata(connection_fifo_tdata),
        .m_axis_tkeep(),

        .s_aclk(tx_axis_aclk),
        .s_aresetn(tx_axis_aresetn),
        .s_axis_tdata({7'b0, s01_axis_rv_lookup_hit, s01_axis_rv_lookup_macAddr, s01_axis_rv_lookup_udpPort, s01_axis_rv_lookup_ipAddr}),
        .s_axis_tkeep({(CONNECTION_META_WIDTH/8){1'b1}}),
        .s_axis_tlast(1'b1),
        .s_axis_tvalid(s01_axis_rv_lookup_valid & tx_engine_enable),
        .s_axis_tready(ready_connection_metadata_fifo)
    );

    fifo_axis_wrapper #(
        .FIFO_DEPTH(128),
        .TDATA_WIDTH(IP_PACKET_LENGTH_WIDTH)
    ) length_metadata_fifo (
        .m_aclk(tx_axis_aclk),
        .m_axis_tready(payload_length_fifo_tready),
        .m_axis_tlast(payload_length_fifo_tlast),
        .m_axis_tvalid(payload_length_fifo_tvalid),
        .m_axis_tdata(payload_length_fifo_tdata),
        .m_axis_tkeep(),

        .s_aclk(tx_axis_aclk),
        .s_aresetn(tx_axis_aresetn),
        .s_axis_tdata(current_total_length_post),
        .s_axis_tkeep({(IP_PACKET_LENGTH_WIDTH/8){1'b1}}),
        .s_axis_tlast(1'b1),
        .s_axis_tvalid(current_total_length_post_valid & tx_engine_enable),
        .s_axis_tready(ready_length_metadata_fifo)
    );


    // ----------------------------------------------------------------------------------------------
    // Length Sum
    // ----------------------------------------------------------------------------------------------

    // Note: the packet fifo is the one that supposed to fill first
    // so no case where we drop a length or connection metadata because the backpressure is not handles
    assign udp_tx_axis_tready = ready_packet_fifo;

    always_ff @(posedge tx_axis_aclk) begin
        
        if (!tx_axis_aresetn) begin
            current_total_length            <= 'b0;
            current_total_length_post       <= 'b0;
            current_total_length_post_valid <= 1'b0;
        end else begin

            if (udp_tx_axis_tvalid && udp_tx_axis_tready) begin
                if (udp_tx_axis_tlast) begin
                    current_total_length            <= 'b0;
                    current_total_length_post       <= current_total_length + $countones(udp_tx_axis_tkeep);
                    current_total_length_post_valid <= 1'b1;
                end else begin
                    current_total_length            <= current_total_length + (DATA_WIDTH/8);
                    current_total_length_post       <= 'b0;
                    current_total_length_post_valid <= 1'b0;
                end
            end else begin
                current_total_length_post_valid <= 1'b0;
            end

        end
    end

    // ----------------------------------------------------------------------------------------------
    // ConnectionId Fetch
    // ----------------------------------------------------------------------------------------------

    logic is_first_transaction;

    always_ff @(posedge tx_axis_aclk) begin
        if (!tx_axis_aresetn)
            is_first_transaction <= 1;
        else if (udp_tx_axis_tvalid && udp_tx_axis_tready) begin
            if (udp_tx_axis_tlast)
                is_first_transaction <= 1;
            else
                is_first_transaction <= 0;
        end
    end

    assign m01_axis_rv_lookup_valid         = udp_tx_axis_tvalid & udp_tx_axis_tready & is_first_transaction;
    assign m01_axis_rv_lookup_connectionId  = swap_bytes_4(udp_tx_axis_tdata[31:0])[CONN_ID_WIDTH-1:0];
    assign s01_axis_rv_lookup_ready         = 1'b1;


    // ==============================================================================================
    // Headers Prepend
    // ==============================================================================================

    logic                       to_checksum_tx_axis_tready;
    logic [DATA_WIDTH-1:0]      to_checksum_tx_axis_tdata;
    logic [DATA_WIDTH/8-1:0]    to_checksum_tx_axis_tkeep;
    logic                       to_checksum_tx_axis_tvalid;
    logic                       to_checksum_tx_axis_tlast;

    tx_headers_prepend #(
        .DATA_WIDTH(DATA_WIDTH),
        .CONN_ID_WIDTH(CONN_ID_WIDTH),
        .IP_UDP_DSCP(IP_UDP_DSCP),
        .IP_UDP_ENC(IP_UDP_ENC),
        .IP_UDP_IDEN(IP_UDP_IDEN),
        .IP_UDP_FLAGS(IP_UDP_FLAGS),
        .IP_UDP_FRAG_OFFSET(IP_UDP_FRAG_OFFSET),
        .IP_UDP_TTL(IP_UDP_TTL)
    ) tx_headers_prepend_unit (
        .tx_axis_aclk(tx_axis_aclk),
        .tx_axis_aresetn(tx_axis_aresetn),
        .my_config_ipAddr(my_config_ipAddr),
        .my_config_macAddr(my_config_macAddr),
        .my_config_udpPort(my_config_udpPort),

        .packet_fifo_tlast(packet_fifo_tlast),
        .packet_fifo_tvalid(packet_fifo_tvalid),
        .packet_fifo_tdata(packet_fifo_tdata),
        .packet_fifo_tkeep(packet_fifo_tkeep),
        .packet_fifo_tready(packet_fifo_tready),

        .connection_fifo_tlast(connection_fifo_tlast),
        .connection_fifo_tvalid(connection_fifo_tvalid),
        .connection_fifo_tdata(connection_fifo_tdata),
        .connection_fifo_tready(connection_fifo_tready),

        .payload_length_fifo_tlast(payload_length_fifo_tlast),
        .payload_length_fifo_tvalid(payload_length_fifo_tvalid),
        .payload_length_fifo_tdata(payload_length_fifo_tdata),
        .payload_length_fifo_tready(payload_length_fifo_tready),

        .to_checksum_tx_axis_tready(to_checksum_tx_axis_tready),
        .to_checksum_tx_axis_tdata(to_checksum_tx_axis_tdata),
        .to_checksum_tx_axis_tkeep(to_checksum_tx_axis_tkeep),
        .to_checksum_tx_axis_tvalid(to_checksum_tx_axis_tvalid),
        .to_checksum_tx_axis_tlast(to_checksum_tx_axis_tlast)
    );


    // ==============================================================================================
    // Checksum computation (TODO)
    // ==============================================================================================

    logic                       from_checksum_tx_axis_tready;
    logic [DATA_WIDTH-1:0]      from_checksum_tx_axis_tdata;
    logic [DATA_WIDTH/8-1:0]    from_checksum_tx_axis_tkeep;
    logic                       from_checksum_tx_axis_tvalid;
    logic                       from_checksum_tx_axis_tlast;

    always_comb begin
        to_checksum_tx_axis_tready      = from_checksum_tx_axis_tready;
        
        from_checksum_tx_axis_tdata     = to_checksum_tx_axis_tdata;
        from_checksum_tx_axis_tkeep     = to_checksum_tx_axis_tkeep;
        from_checksum_tx_axis_tvalid    = to_checksum_tx_axis_tvalid;
        from_checksum_tx_axis_tlast     = to_checksum_tx_axis_tlast; 
    end


    // ==============================================================================================
    // Output
    // ==============================================================================================

    always_comb begin
        cmac_tx_axis_tdata  = from_checksum_tx_axis_tdata;
        cmac_tx_axis_tkeep  = from_checksum_tx_axis_tkeep;
        cmac_tx_axis_tvalid = from_checksum_tx_axis_tvalid;
        cmac_tx_axis_tlast  = from_checksum_tx_axis_tlast;
        from_checksum_tx_axis_tready = cmac_tx_axis_tready;
    end

endmodule
`default_nettype wire