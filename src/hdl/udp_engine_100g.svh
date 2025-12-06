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

function automatic logic [15:0] hash_fun_ip_port (
    input logic [31:0] ip,
    input logic [15:0] port
);
    logic [47:0] key;
    logic [31:0] mix;

    key = {ip, port};  // 48 bits

    // Fold down to 32 bits with some rotation-ish mixing
    mix = key[31:0] ^ {key[47:32], key[15:0]};

    // Multiplicative hash (odd constant, like Knuth's)
    mix = mix * 32'h9E3779B1;

    // Take upper 16 bits (usually better than lower)
    return mix[31:16];
endfunction


function automatic logic [15:0] swap_bytes_2(input logic [15:0] din);
    swap_bytes_2 = {din[7:0], din[15:8]};
endfunction

function automatic logic [31:0] swap_bytes_4(input logic [31:0] din);
    for (int i = 0; i < 4; i++) begin
        swap_bytes_4[i*8 +: 8] = din[(4-1-i)*8 +: 8];
    end
endfunction

function automatic logic [47:0] swap_bytes_6(input logic [47:0] din);
    for (int i = 0; i < 6; i++) begin
        swap_bytes_6[i*8 +: 8] = din[(6-1-i)*8 +: 8];
    end
endfunction




`endif  // UDP_ENGINE_100G_HEADER

