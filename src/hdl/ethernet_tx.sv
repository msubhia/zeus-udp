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
// tx_engine_bypass:    if asserted, payloads are bypassed without packet headers and formating
// tx_engine_enable:    if de-asserted, incoming payloads are dropped
// 
// ============================================================================
// END
// ============================================================================

module ethernet_tx #(
    parameter int DATA_WIDTH                    = 512,
    parameter int CONN_ID_WIDTH                 = 18,
    parameter int IP_UDP_DSCP                   = 0,
    parameter int IP_UDP_ENC                    = 0,
    parameter int IP_UDP_IDEN                   = 0,
    parameter int IP_UDP_FLAGS                  = 0,
    parameter int IP_UDP_FRAG_OFFSET            = 0,
    parameter int IP_UDP_TTL                    = 64,
) (
    // ----------------------------------------------------------------
    // CONTROL AND STATUS
    // ----------------------------------------------------------------
    input wire                          tx_axis_aclk,
    input wire                          tx_axis_aresetn,
    input wire                          tx_engine_bypass,   // if asserted, payloads are bypassed without packet headers and formating
    input wire                          tx_engine_enable,   // if de-asserted, incoming payloads are dropped

    // ----------------------------------------------------------------
    // CONNECTION CONFIGURATION
    // ----------------------------------------------------------------
    input wire [IP_ADDR_WIDTH-1:0]      my_config_ipAddr,
    input wire [MAC_ADDR_WIDTH-1:0]     my_config_macAddr,
    input wire [UDP_PORT_WIDTH-1:0]     my_config_udpPort,

    // ----------------------------------------------------------------
    // CMAC TX
    // ----------------------------------------------------------------
    input wire                      cmac_tx_axis_tready,
    output logic [DATA_WIDTH-1:0]   cmac_tx_axis_tdata,
    output logic [DATA_WIDTH/8-1:0] cmac_tx_axis_tkeep,
    output logic                    cmac_tx_axis_tvalid,
    output logic                    cmac_tx_axis_tlast,

    // ----------------------------------------------------------------
    // USER INPUT
    // ----------------------------------------------------------------
    output logic                    udp_tx_axis_tready,
    input wire [DATA_WIDTH-1:0]     udp_tx_axis_tdata,
    input wire [DATA_WIDTH/8-1:0]   udp_tx_axis_tkeep,
    input wire                      udp_tx_axis_tvalid,
    input wire                      udp_tx_axis_tlast,

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

    // As packets are entering the fifo we count their length; once an entire packet enters the fifo
    // we append its length along with the destination connection information in a metadata fifo to be
    // be used by an FSM at the consuming part to start constructing the packet and frame it.

    localparam int IP_ADDR_WIDTH    = 32,
    localparam int MAC_ADDR_WIDTH   = 48,
    localparam int UDP_PORT_WIDTH   = 16,
    localparam int TOTAL_UDP_HEADERS        = IP_ADDR_WIDTH + MAC_ADDR_WIDTH + UDP_PORT_WIDTH;
    localparam int RPC_HEADER               = 64;
    localparam int IP_PACKET_LENGTH_WIDTH   = 16;

    logic ready_packet_fifo;
    logic ready_connection_metadata_fifo;
    logic ready_length_metadata_fifo;

    logic m_packet_fifo_tready;
    logic m_packet_fifo_tlast;
    logic m_packet_fifo_tvalid;
    logic m_packet_fifo_tdata;
    logic m_packet_fifo_tkeep;

    logic m_connection_fifo_tready;
    logic m_connection_fifo_tlast;
    logic m_connection_fifo_tvalid;
    logic m_connection_fifo_tdata;
    logic m_connection_fifo_tkeep;

    logic m_lenth_fifo_tready;
    logic m_lenth_fifo_tlast;
    logic m_lenth_fifo_tvalid;
    logic m_lenth_fifo_tdata;
    logic m_lenth_fifo_tkeep;

    fifo_axis_wrapper #(
        .FIFO_DEPTH(64),
        .TDATA_WIDTH(DATA_WIDTH)
    ) packet_fifo (
        .m_aclk(tx_axis_aclk),
        .m_axis_tready(m_packet_fifo_tready),
        .m_axis_tlast(m_packet_fifo_tlast),
        .m_axis_tvalid(m_packet_fifo_tvalid),
        .m_axis_tdata(m_packet_fifo_tdata),
        .m_axis_tkeep(m_packet_fifo_tkeep),

        .s_aclk(tx_axis_aclk),
        .s_aresetn(tx_axis_aresetn),
        .s_axis_tdata(udp_tx_axis_tdata),
        .s_axis_tkeep(udp_tx_axis_tkeep),
        .s_axis_tlast(udp_tx_axis_tlast),
        .s_axis_tvalid(udp_tx_axis_tvalid & tx_engine_enable),
        .s_axis_tready(ready_packet_fifo)
    );

    fifo_axis_wrapper #(
        .FIFO_DEPTH(64),
        .TDATA_WIDTH(TOTAL_UDP_HEADERS + 1 /*hit bit*/)
    ) connection_metadata_fifo (
        .m_aclk(tx_axis_aclk),
        .m_axis_tready(m_connection_fifo_tready),
        .m_axis_tlast(m_connection_fifo_tlast),
        .m_axis_tvalid(m_connection_fifo_tvalid),
        .m_axis_tdata(m_connection_fifo_tdata),
        .m_axis_tkeep(m_connection_fifo_tkeep),

        .s_aclk(tx_axis_aclk),
        .s_aresetn(tx_axis_aresetn),
        .s_axis_tdata({s01_axis_rv_lookup_hit, s01_axis_rv_lookup_macAddr, s01_axis_rv_lookup_udpPort, s01_axis_rv_lookup_ipAddr}),
        .s_axis_tkeep({(TOTAL_UDP_HEADERS/8 + 1){1'b1}}),
        .s_axis_tlast(1'b1),
        .s_axis_tvalid(s01_axis_rv_lookup_valid & tx_engine_enable),
        .s_axis_tready(ready_connection_metadata_fifo)
    );

    fifo_axis_wrapper #(
        .FIFO_DEPTH(64),
        .TDATA_WIDTH(IP_PACKET_LENGTH_WIDTH)
    ) length_metadata_fifo (
        .m_aclk(tx_axis_aclk),
        .m_axis_tready(m_lenth_fifo_tready),
        .m_axis_tlast(m_lenth_fifo_tlast),
        .m_axis_tvalid(m_lenth_fifo_tvalid),
        .m_axis_tdata(m_lenth_fifo_tdata),
        .m_axis_tkeep(m_lenth_fifo_tkeep),

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
    assign udp_tx_axis_tready = ready_packet_fifo & ready_connection_metadata_fifo & ready_length_metadata_fifo;

    logic [IP_PACKET_LENGTH_WIDTH-1:0]      current_total_length;
    logic [IP_PACKET_LENGTH_WIDTH-1:0]      current_total_length_post;
    logic                                   current_total_length_post_valid;

    always_ff @(posedge tx_axis_aclk) begin
        
        if (!tx_axis_aresetn) begin
            current_total_length            <= (TOTAL_HEADERS-RPC_HEADER)/8;
            current_total_length_post       <= 15'b0;
            current_total_length_post_valid <= 1'b0;
        end else begin

            if (udp_tx_axis_tvalid && udp_tx_axis_tready) begin
                if (udp_tx_axis_tlast) begin
                    current_total_length            <= (TOTAL_HEADERS-RPC_HEADER)/8;
                    current_total_length_post       <= current_total_length + $countones(udp_tx_axis_tkeep);
                    current_total_length_post_valid <= 1'b1;
                end else begin
                    current_total_length            <= current_total_length + (TOTAL_HEADERS-RPC_HEADER)/8;
                    current_total_length_post       <= 15'b0;
                    current_total_length_post_valid <= 1'b0;
                end
            end

        end
    end

    // ----------------------------------------------------------------------------------------------
    // ConnectionId Fetch
    // ----------------------------------------------------------------------------------------------

    logic is_first_transaction;

    always_ff @(posedge tx_axis_aclk) begin
        if (!tx_axis_aresetn) begin
            is_first_transaction            <= 1'b1;
        end else begin
            if (udp_tx_axis_tvalid && udp_tx_axis_tready) begin
                if (udp_tx_axis_tlast) begin
                    is_first_transaction            <= 1'b1;
                end else begin
                    is_first_transaction            <= 1'b0;
                end
            end
        end
    end

    assign m01_axis_rv_lookup_valid         = udp_tx_axis_tvalid & udp_tx_axis_tready & is_first_transaction;
    assign m01_axis_rv_lookup_connectionId  = udp_tx_axis_tdata[CONN_ID_WIDTH-1:0];
    assign s01_axis_rv_lookup_ready         = 1'b1;



    // ==============================================================================================
    // Consuming FSM
    // ==============================================================================================

    // waits until both metadata fifos has information and triggers: should be deeply pipelined
    // it preappends headers and does their checksum


    // ----------------------------------------------------------------------------------------------
    // HEADERS Construction
    // ----------------------------------------------------------------------------------------------

    function automatic logic [8*NUM_BYTES-1:0] swap_bytes
        #(int NUM_BYTES = 4)
        (input logic [8*NUM_BYTES-1:0] din);

        for (int i = 0; i < NUM_BYTES; i++) begin
            swap_bytes[i*8 +: 8] = din[(NUM_BYTES-1-i)*8 +: 8];
        end
    endfunction


    // Assumes incoming payloads are RPC Packets with connectionId is the first 32-bits
    // the module uses this connectionId to fetch destination macAddr, ipAddr, udpPort
    // the fetched results will be ready after 3 cycles

    // Ethernet Header (14 bytes = 112 bits)
    //      - [47:0] dst_addr;
    //      - [47:0] src_addr;
    //      - [15:0] eth_type;

    logic [111:0] my_eth_header;
    always_comb begin
        my_eth_header[47:0]    = swap_bytes #(.NUM_BYTES(6))();
        my_eth_header[95:48]   = swap_bytes #(.NUM_BYTES(6))(my_config_macAddr);
        my_eth_header[111:96]  = swap_bytes #(.NUM_BYTES(2))(ETHTYPE_IP);
    end

    // Ip Header (20 bytes = 160 bits)
    //      - [3:0]  version;
    //      - [3:0]  header_length;  // Header length in 32-bit words
    //      - [5:0]  dscp;           // QoS, traffic class
    //      - [1:0]  ecn;            // Congestion notification
    //      - [15:0] total_length;   // Full IP length including header & payload

    //      - [15:0] identification;  // For fragmentation
    //      - [2:0]  flags;           // DF, MF flags
    //      - [12:0] frag_offset;     // Where this fragment fits
    //      - [7:0]  ttl;             // Time-to-live

    //      - [7:0]  protocol;      // protocol
    //      - [15:0] hdr_checksum;  // Checksum of IPv4 header
    //      - [31:0] src_ip;        // Sender IPv4 address
    //      - [31:0] dst_ip;        // Receiver IPv4 address

    logic [159:0] my_ip_header;
    always_comb begin
        // constant
        my_ip_header[7:4]       = IP_VERSION_IPV4;
        my_ip_header[3:0]       = IP_HEADER_BYTES;
        my_ip_header[15:10]     = IP_UDP_DSCP;
        my_ip_header[9:8]       = IP_UDP_ECN;
        my_ip_header[47:32]     = swap_bytes#(.NUM_BYTES(2))(IP_UDP_IDEN);
        my_ip_header[63:48]     = swap_bytes#(.NUM_BYTES(2))({IP_UDP_FLAGS, IP_UDP_FRAG_OFFSET});
        my_ip_header[71:64]     = IP_TTL;
        my_ip_header[79:72]     = IPPROTO_UDP;

        // varies with packet
        my_ip_header[31:16]     = swap_bytes#(.NUM_BYTES(2))(/*todo total_length */);
        my_ip_header[95:80]     = swap_bytes#(.NUM_BYTES(2))(/*todo hdr_checksum */);
        my_ip_header[127:96]    = swap_bytes#(.NUM_BYTES(4))(my_config_ipAddr);
        my_ip_header[159:128]   = swap_bytes#(.NUM_BYTES(4))();
    end

    // UDP Header (8 bytes = 64 bits)
    //      - src_port;  // Sender's port
    //      - dst_port;  // Receiver's port
    //      - length;    // length, including udp header, not including IP header
    //      - checksum;  // unused in IPv4

    logic [63:0] my_udp_header;
    always_comb begin
        my_udp_header[15:0]     = swap_bytes#(.NUM_BYTES(2))(my_config_udpPort);
        my_udp_header[31:16]    = swap_bytes#(.NUM_BYTES(2))();
        my_udp_header[47:32]    = swap_bytes#(.NUM_BYTES(2))(/*todo length */);
        my_udp_header[63:48]    = swap_bytes#(.NUM_BYTES(2))(/*todo checksum */);
    end

    localparam TOTAL_HEADERS    = 336;
    localparam REMAINING_TAIL   = DATA_WIDTH - TOTAL_HEADERS;


    // ----------------------------------------------------------------------------------------------
    // HEADERS Construction
    // ----------------------------------------------------------------------------------------------

    // cmac_tx_axis_tready
    // cmac_tx_axis_tdata
    // cmac_tx_axis_tkeep
    // cmac_tx_axis_tvalid
    // cmac_tx_axis_tlast

    // logic m_packet_fifo_tready
    // logic m_packet_fifo_tlast
    // logic m_packet_fifo_tvalid
    // logic m_packet_fifo_tdata
    // logic m_packet_fifo_tkeep

    // logic m_connection_fifo_tready
    // logic m_connection_fifo_tlast
    // logic m_connection_fifo_tvalid
    // logic m_connection_fifo_tdata
    // logic m_connection_fifo_tkeep

    // logic m_lenth_fifo_tready
    // logic m_lenth_fifo_tlast
    // logic m_lenth_fifo_tvalid
    // logic m_lenth_fifo_tdata
    // logic m_lenth_fifo_tkeep

    logic new_packet_available;
    assign new_packet_available = m_packet_fifo_tvalid & m_connection_fifo_tvalid & m_lenth_fifo_tvalid;

    logic m_fifo_ready;
    assign m_packet_fifo_tready     = m_fifo_ready;
    assign m_connection_fifo_tready = m_fifo_ready;
    assign m_lenth_fifo_tready      = m_fifo_ready;

    always_ff @(posedge tx_axis_aclk) begin

        if (!tx_axis_aresetn) begin
            cmac_tx_axis_tdata  <= 'b0;
            cmac_tx_axis_tkeep  <= 'b0;
            cmac_tx_axis_tvalid <= 'b0;
            cmac_tx_axis_tlast  <= 'b0;

            m_fifo_ready        <= 'b0;

        end else begin
            // logic is similar to prepend

        end
    end
    
endmodule