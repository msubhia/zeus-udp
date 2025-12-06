`default_nettype none
`timescale 1ns/1ps

`include "udp_engine_100g.svh"

// ==================================================================================================
// TX Ethernet, IP, UDP Headers Prepend IP
// ==================================================================================================
//
// Author:          M.Subhi Abordan (msubhi_a@mit.edu)
//                  Mena Filfil     (menaf@mit.edu)
// Last Modified:   Dec 5, 2025
//
//==================================================================================================
//
// Takes a fifo of packets as well two other fifos, connection metadata, and 
// length metadata.
// as soon as the metadata fifos both have a field, we start reading a packet
// until tlast and prepend with ethernet, ip and udp headers with checksums begin 0
// to be computed by downsteam module
// 
// assumes downstream module always ready
//
//==================================================================================================
//
// Ethernet Header (14 bytes = 112 bits)
//      - [47:0] dst_addr;
//      - [47:0] src_addr;
//      - [15:0] eth_type;
//
// Ip Header (20 bytes = 160 bits)
//      - [3:0]  version;
//      - [3:0]  header_length;  // Header length in 32-bit words
//      - [5:0]  dscp;           // QoS, traffic class
//      - [1:0]  ecn;            // Congestion notification
//      - [15:0] total_length;   // Full IP length including header & payload
//
//      - [15:0] identification;  // For fragmentation
//      - [2:0]  flags;           // DF, MF flags
//      - [12:0] frag_offset;     // Where this fragment fits
//      - [7:0]  ttl;             // Time-to-live
//
//      - [7:0]  protocol;      // protocol
//      - [15:0] hdr_checksum;  // Checksum of IPv4 header
//      - [31:0] src_ip;        // Sender IPv4 address
//      - [31:0] dst_ip;        // Receiver IPv4 address
//
// UDP Header (8 bytes = 64 bits)
//      - src_port;  // Sender's port
//      - dst_port;  // Receiver's port
//      - length;    // length, including udp header, not including IP header
//      - checksum;  // unused in IPv4
//
//
// ==================================================================================================
// END
// ==================================================================================================


module tx_headers_prepend #(
    parameter int DATA_WIDTH    = 512,
    parameter int CONN_ID_WIDTH
) (
    input wire                                  tx_axis_aclk,
    input wire                                  tx_axis_aresetn,

    input wire [MAC_ADDR_WIDTH-1:0]             my_config_dst_macAddr,
    input wire [MAC_ADDR_WIDTH-1:0]             my_config_src_macAddr,
    input wire [IP_ADDR_WIDTH-1:0]              my_config_src_ipAddr,
    input wire [UDP_PORT_WIDTH-1:0]             my_config_src_udpPort,

    input wire                                  packet_fifo_tlast,
    input wire                                  packet_fifo_tvalid,
    input wire [DATA_WIDTH  -1:0]               packet_fifo_tdata,
    input wire [DATA_WIDTH/8-1:0]               packet_fifo_tkeep,
    output logic                                packet_fifo_tready,

    input wire                                  connection_fifo_tlast,
    input wire                                  connection_fifo_tvalid,
    input wire [CONNECTION_META_WIDTH-1:0]      connection_fifo_tdata,
    output logic                                connection_fifo_tready,

    input wire                                  payload_length_fifo_tlast,
    input wire                                  payload_length_fifo_tvalid,
    input wire [IP_PACKET_LENGTH_WIDTH-1:0]     payload_length_fifo_tdata,
    output logic                                payload_length_fifo_tready,

    input wire                                  to_checksum_tx_axis_tready, // assumes always ready
    output logic [DATA_WIDTH  -1:0]             to_checksum_tx_axis_tdata,
    output logic [DATA_WIDTH/8-1:0]             to_checksum_tx_axis_tkeep,
    output logic                                to_checksum_tx_axis_tvalid,
    output logic                                to_checksum_tx_axis_tlast
);
    // parsing connection metadata
    logic                       dst_connection_hit;
    logic [IP_ADDR_WIDTH-1:0]   dst_ipAddr;
    logic [UDP_PORT_WIDTH-1:0]  dst_udpPort;

    assign dst_connection_hit   = connection_fifo_tdata[48];
    assign dst_ipAddr           = connection_fifo_tdata[31:0];
    assign dst_udpPort          = connection_fifo_tdata[47:32];

    // ==================================================================================================
    // Headers Construction: Big Endian
    // ==================================================================================================

    logic [111:0] my_eth_header;
    logic [159:0] my_ip_header;
    logic [63:0] my_udp_header;

    always_comb begin: HEADERS_CONSTRUCTION
        my_eth_header[47:0]     = swap_bytes_6(my_config_dst_macAddr);
        my_eth_header[95:48]    = swap_bytes_6(my_config_src_macAddr);
        my_eth_header[111:96]   = swap_bytes_2(ETHTYPE_IP);

        my_ip_header[7:4]       = IP_VERSION_IPV4;
        my_ip_header[3:0]       = IP_HEADER_BYTES/4; // in 32-words
        my_ip_header[15:10]     = IP_UDP_DSCP;
        my_ip_header[9:8]       = IP_UDP_ENC;
        my_ip_header[47:32]     = swap_bytes_2(IP_UDP_IDEN);
        my_ip_header[63:48]     = swap_bytes_2({IP_UDP_FLAGS, IP_UDP_FRAG_OFFSET});
        my_ip_header[71:64]     = IP_UDP_TTL;
        my_ip_header[79:72]     = IPPROTO_UDP;
        my_ip_header[31:16]     = swap_bytes_2(payload_length_fifo_tdata + IP_HEADER_BYTES + UDP_HEADER_BYTES);
        my_ip_header[95:80]     = 16'b0; // ip_hdr_checksum to be computed by downstream module
        my_ip_header[127:96]    = swap_bytes_4(my_config_src_ipAddr);
        my_ip_header[159:128]   = swap_bytes_4(dst_ipAddr);

        my_udp_header[15:0]     = swap_bytes_2(my_config_src_udpPort);
        my_udp_header[31:16]    = swap_bytes_2(dst_udpPort);
        my_udp_header[47:32]    = swap_bytes_2(payload_length_fifo_tdata + UDP_HEADER_BYTES);
        my_udp_header[63:48]    = 16'b0; // udp_checksum optional in IPv4
    end

    logic [TOTAL_HEADERS_BITS-1:0] my_headers_draft;
    assign my_headers_draft = {my_udp_header, my_ip_header, my_eth_header};


    // ==================================================================================================
    // Prepending Headers
    // ==================================================================================================

    typedef enum logic[1:0] {TX_TAIL, TX_BODY, TX_HEAD} tx_state_t;
    tx_state_t my_tx_state;

    logic new_packet_available; // a packet meta-data and it's head available
    assign new_packet_available = connection_fifo_tvalid & payload_length_fifo_tvalid & packet_fifo_tvalid;

    logic is_head, is_tail;
    assign is_head = (my_tx_state == TX_HEAD);
    assign is_tail = (my_tx_state == TX_TAIL);

    assign packet_fifo_tready           = tx_axis_aresetn && (!is_tail) && (!is_head || new_packet_available);
    assign connection_fifo_tready       = tx_axis_aresetn && (!is_tail) && is_head && new_packet_available;
    assign payload_length_fifo_tready   = tx_axis_aresetn && (!is_tail) && is_head && new_packet_available;

    logic [DATA_WIDTH/8-1:0] to_checksum_tx_axis_tkeep_tail;
    logic [TOTAL_HEADERS_BITS-1:0] internal_reg;
    logic dst_connection_hit_hold;

    always_ff @(posedge tx_axis_aclk) begin

        if (!tx_axis_aresetn) begin
            my_tx_state                     <= TX_HEAD;
            internal_reg                    <= 'b0;
            to_checksum_tx_axis_tkeep_tail  <= 'b0;

            to_checksum_tx_axis_tdata       <= 'b0;
            to_checksum_tx_axis_tkeep       <= 'b0;
            to_checksum_tx_axis_tvalid      <= 1'b0;
            to_checksum_tx_axis_tlast       <= 1'b0;
            dst_connection_hit_hold         <= 1'b0;

        end else begin
            case (my_tx_state)

                TX_HEAD: begin
                    if (packet_fifo_tvalid && packet_fifo_tready) begin
                        dst_connection_hit_hold     <= dst_connection_hit;
                        to_checksum_tx_axis_tvalid  <= dst_connection_hit;
                        to_checksum_tx_axis_tdata   <= {packet_fifo_tdata[ (DATA_WIDTH - TOTAL_HEADERS_BITS - 1) : 0 ], my_headers_draft};
                        internal_reg                <= packet_fifo_tdata[ (DATA_WIDTH-1) : (DATA_WIDTH - TOTAL_HEADERS_BITS) ];

                        if (packet_fifo_tlast) begin
                            if (packet_fifo_tkeep > {{TOTAL_HEADERS_BYTES{1'b0}}, {(DATA_WIDTH/8-TOTAL_HEADERS_BYTES){1'b1}}}) begin
                                my_tx_state                     <= TX_TAIL;
                                to_checksum_tx_axis_tkeep       <= {(DATA_WIDTH/8){1'b1}};
                                to_checksum_tx_axis_tlast       <= 1'b0;
                                to_checksum_tx_axis_tkeep_tail  <= (packet_fifo_tkeep >> (DATA_WIDTH/8-TOTAL_HEADERS_BYTES));
                            end else begin
                                my_tx_state                     <= TX_HEAD;
                                to_checksum_tx_axis_tkeep       <= (packet_fifo_tkeep << TOTAL_HEADERS_BYTES) | {(TOTAL_HEADERS_BYTES){1'b1}};
                                to_checksum_tx_axis_tlast       <= 1'b1;
                            end
                        end else begin
                            my_tx_state                         <= TX_BODY;
                            to_checksum_tx_axis_tkeep           <= {(DATA_WIDTH/8){1'b1}};
                            to_checksum_tx_axis_tlast           <= 1'b0;
                        end

                    end else begin // pause waiting for packets
                        to_checksum_tx_axis_tvalid      <= 1'b0;
                    end
                end

                // ---------------------------------------------------------------------------------------------------

                TX_BODY: begin
                    if (packet_fifo_tvalid && packet_fifo_tready) begin

                        to_checksum_tx_axis_tvalid  <= dst_connection_hit_hold;
                        to_checksum_tx_axis_tdata   <= {packet_fifo_tdata[ (DATA_WIDTH - TOTAL_HEADERS_BITS - 1) : 0 ], internal_reg};
                        internal_reg                <= packet_fifo_tdata[ (DATA_WIDTH-1) : (DATA_WIDTH - TOTAL_HEADERS_BITS) ];

                        if (packet_fifo_tlast) begin
                            if (packet_fifo_tkeep > {{TOTAL_HEADERS_BYTES{1'b0}}, {(DATA_WIDTH/8-TOTAL_HEADERS_BYTES){1'b1}}}) begin
                                my_tx_state                     <= TX_TAIL;
                                to_checksum_tx_axis_tkeep       <= {(DATA_WIDTH/8){1'b1}};
                                to_checksum_tx_axis_tlast       <= 1'b0;
                                to_checksum_tx_axis_tkeep_tail  <= (packet_fifo_tkeep >> (DATA_WIDTH/8-TOTAL_HEADERS_BYTES));
                            end else begin
                                my_tx_state                     <= TX_HEAD;
                                to_checksum_tx_axis_tkeep       <= (packet_fifo_tkeep << TOTAL_HEADERS_BYTES) | {(TOTAL_HEADERS_BYTES){1'b1}};
                                to_checksum_tx_axis_tlast       <= 1'b1;
                            end
                        end else begin
                            my_tx_state                         <= TX_BODY;
                            to_checksum_tx_axis_tkeep           <= {(DATA_WIDTH/8){1'b1}};
                            to_checksum_tx_axis_tlast           <= 1'b0;
                        end

                    end else begin // pause in middle of packet
                        to_checksum_tx_axis_tvalid      <= 1'b0;
                    end
                end

                // ---------------------------------------------------------------------------------------------------
                TX_TAIL: begin
                    my_tx_state                     <= TX_HEAD;
                    to_checksum_tx_axis_tvalid      <= dst_connection_hit_hold;
                    to_checksum_tx_axis_tdata       <= {{(DATA_WIDTH-TOTAL_HEADERS_BITS){1'b0}}, internal_reg};
                    to_checksum_tx_axis_tkeep       <= to_checksum_tx_axis_tkeep_tail;
                    to_checksum_tx_axis_tlast       <= 1'b1;
                end
                // ---------------------------------------------------------------------------------------------------

            endcase

        end
    end


endmodule // tx_headers_prepend
`default_nettype wire