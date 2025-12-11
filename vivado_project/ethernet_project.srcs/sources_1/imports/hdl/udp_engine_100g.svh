`ifndef UDP_ENGINE_100G_HEADER
`define UDP_ENGINE_100G_HEADER

// ==========================================================================================
// Networking Constants
// ==========================================================================================
parameter IP_ADDR_WIDTH             = 32;
parameter MAC_ADDR_WIDTH            = 48;
parameter UDP_PORT_WIDTH            = 16;

parameter ETHTYPE_IP                = 16'h0800;     // Internet protocol
parameter ETHTYPE_ARP               = 16'h0806;     // Address resolution protocol
parameter IP_VERSION_IPV4           = 4;
parameter IP_VERSION_IPV6           = 6;
parameter IP_PROTO_ICMP             = 1;            // Control message protocol
parameter IPPROTO_TCP               = 6;            // Transmission control protocol
parameter IPPROTO_UDP               = 17;           // User datagram protocol

parameter IP_PACKET_LENGTH_WIDTH    = 16;
parameter ETH_HEADER_BYTES          = 14;
parameter IP_HEADER_BYTES           = 20;
parameter UDP_HEADER_BYTES          = 8;
parameter TOTAL_HEADERS_BYTES       = ETH_HEADER_BYTES + IP_HEADER_BYTES + UDP_HEADER_BYTES;
parameter TOTAL_HEADERS_BITS        = TOTAL_HEADERS_BYTES * 8;

// ==========================================================================================
// User Inputs
// ==========================================================================================

parameter IP_UDP_DSCP               = 0;
parameter IP_UDP_ENC                = 0;
parameter IP_UDP_IDEN               = 0;
parameter IP_UDP_FLAGS              = 0;
parameter IP_UDP_FRAG_OFFSET        = 0;
parameter IP_UDP_TTL                = 64;

// ==========================================================================================
// Implementation Related
// ==========================================================================================

parameter CONNECTION_META_WIDTH     = IP_ADDR_WIDTH + UDP_PORT_WIDTH + 8;
parameter HASH_WIDTH                = 16;

`endif  // UDP_ENGINE_100G_HEADER
