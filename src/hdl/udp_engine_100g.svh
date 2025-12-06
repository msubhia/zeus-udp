`ifndef UDP_ENGINE_100G_HEADER
`define UDP_ENGINE_100G_HEADER



parameter IP_ADDR_WIDTH    = 32;
parameter MAC_ADDR_WIDTH   = 48;
parameter UDP_PORT_WIDTH   = 16;




parameter HASH_WIDTH       = 16;

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









`endif  // UDP_ENGINE_100G_HEADER

