`ifndef ZEUS_RPC_INCLUDE
`define ZEUS_RPC_INCLUDE

// ----------------------------------------------------------------
// ETHERNET
// ----------------------------------------------------------------
parameter int BYTE_SIZE     = 8;
parameter int MAC_ADDR_SIZE = 6 * BYTE_SIZE;
parameter int ETH_TYPE_SIZE = 2 * BYTE_SIZE;

parameter int ETHTYPE_IP    = 16'h0800 // Internet protocol
parameter int ETHTYPE_ARP   = 16'h0806 // Address resolution protocol

typedef struct packed { // 14 bytes
    // preamble omitted
    logic [MAC_ADDR_SIZE-1:0] dst_addr;
    logic [MAC_ADDR_SIZE-1:0] src_addr;
    logic [ETH_TYPE_SIZE-1:0] eth_type;
} ethernet_header_t;


// ----------------------------------------------------------------
// IP
// ----------------------------------------------------------------
parameter int IP_VERSION_IPV4   = 4;
parameter int IP_VERSION_IPV6   = 6;

parameter int IP_PROTO_ICMP     = 1     // Control message protocol
parameter int IPPROTO_TCP       = 6     // Transmission control protocol
parameter int IPPROTO_UDP       = 17    // User datagram protocol

typedef struct packed { // 20 bytes
    logic [3:0]   version;
    logic [3:0]   header_length;    // Header length in 32-bit words
    logic [5:0]   dscp;             // QoS, traffic class
    logic [1:0]   ecn;              // Congestion notification
    logic [15:0]  total_length;     // Full IP length including header & payload

    logic [15:0]  identification;   // For fragmentation
    logic [2:0]   flags;            // DF, MF flags
    logic [12:0]  frag_offset;      // Where this fragment fits
    logic [7:0]   ttl;              // Time-to-live

    logic [7:0]   protocol;         // protocol
    logic [15:0]  hdr_checksum;     // Checksum of IPv4 header
    logic [31:0]  src_ip;           // Sender IPv4 address
    logic [31:0]  dst_ip;           // Receiver IPv4 address

    // options omitted
} ipv4_header_t;


// typedef struct packed { // 20 bytes
//     logic [3:0]     version;
//     logic [3:0]     header_length;    // Header length in 32-bit words
//     logic [63:0]    irrelavent;
//     logic [7:0]     protocol;         // protocol
//     logic [15:0]    hdr_checksum;     // Checksum of IPv4 header
//     logic [31:0]    src_ip;           // Sender IPv4 address
//     logic [31:0]    dst_ip;           // Receiver IPv4 address
//     // options omitted
// } ipv4_header_simple_t;


// ----------------------------------------------------------------
// UDP
// ----------------------------------------------------------------

typedef struct packed {     // 8 bytes
    logic [15:0] src_port;  // Sender's port
    logic [15:0] dst_port;  // Receiver's port
    logic [15:0] length;    // length, including udp header, not including IP header
    logic [15:0] checksum;  // unused in IPv4
} udp_header_t;



// ----------------------------------------------------------------
// RPC
// ----------------------------------------------------------------

typedef struct packed { // 128 bytes
    logic [31:0]    connection_id;  // an id for indexing client MAC, IP, Port
    logic [31:0]    pckt_id;        // id, timestamp, seqnum, ...
    logic [479:0]   pckt_load;      // should include meta-data for parsing
} rpc_pckt_t;



// ----------------------------------------------------------------
// Connection Configuration
// ----------------------------------------------------------------

typedef struct packed {
    logic [MAC_ADDR_SIZE-1:0]   mac_addr;
    logic [31:0]                ip_addr;
} connection_config_t;




`endif // ZEUS_RPC_INCLUDE