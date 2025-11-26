`default_nettype none



//
// Processes Ethernet Packets
// 
// Checks if valid connection: our (dest) mac/ip/port and client mac/ip/port if a registered user
// drops any packets from unregistered connections
// drop non-valid checksums
//
// ethernet header              = 14 bytes
// ip header                    = 20 bytes
// udp header                   = 8  bytes
//-----------------------------
// network layer total          = 42 bytes
// 
// total interface size = minimum packet size = 64 bytes.
// so we could be recieving a packet each cycle, the pipeline should be good enough to not drop anything
// the packet could be also divided over multiple transactions
// 
//
// consumer recives an un-parsed RPC packet which consists of connection id and udp packet payload
//
//
// out dest mac addr, ip addr, reciving port are setup by user through axi-lite and are constant 
// connection manager handles book-keeping clients (mac, ip, port)-> (connection_id)
// maybe we have a maximum number of connections
//
// to solve: invalidating connections while packets in fly
//
// this module does not stall even if the downstream module is busy. so ready signals are ignored.
//
//
//
// Suppose the wire sends:
// Dest MAC:   11 22 33 44 55 66
// Src MAC:    AA BB CC DD EE FF
// Ethertype:  08 00


// Then AXIS will present:

// tdata[ 7:0]   = 0x11   // Dest MAC byte 0
// tdata[15:8]   = 0x22
// tdata[23:16]  = 0x33
// tdata[31:24]  = 0x44
// tdata[39:32]  = 0x55
// tdata[47:40]  = 0x66

// tdata[55:48]  = 0xAA   // Src MAC byte 0
// tdata[63:56]  = 0xBB
// tdata[71:64]  = 0xCC
// tdata[79:72]  = 0xDD
// tdata[87:80]  = 0xEE
// tdata[95:88]  = 0xFF

// tdata[103:96] = 0x08   // Ethertype byte 0
// tdata[111:104]= 0x00

module ethernet_rx #(
    parameter int DATA_WIDTH = 512,
    parameter int KEEP_WIDTH = DATA_WIDTH/8
) (
    // ----------------------------------------------------------------
    // CMAC RX INFERFACE
    // ----------------------------------------------------------------
    input  logic                    aclk,
    input  logic                    aresetn,

    //////// RX ////////
    input  logic [DATA_WIDTH-1:0]   s_axis_tdata,
    input  logic [KEEP_WIDTH-1:0]   s_axis_tkeep,
    input  logic                    s_axis_tvalid,
    output logic                    s_axis_tready,
    input  logic                    s_axis_tlast,
    input  logic [0:0]              s_axis_tuser,

    // ----------------------------------------------------------------
    // OUTPUT
    // ----------------------------------------------------------------
    output logic [DATA_WIDTH-1:0]   m_axis_tdata,
    output logic [KEEP_WIDTH-1:0]   m_axis_tkeep,
    output logic                    m_axis_tvalid,
    input  logic                    m_axis_tready,
    output logic                    m_axis_tlast,
    output logic [0:0]              m_axis_tuser,

    input connection_config_t my_config
);

    logic [MAC_ADDR_SIZE-1:0]   my_mac_addr = my_config.mac_addr;
    logic [31:0]                my_ip_addr  = my_config.my_ip_addr;


    // in a single cycle:
    // 1- my mac == dest mac
    // 2- eth type is ip
    // 3- ip version is not 4
    // 4- my ip = dest ip
    // 5- ip protocol is udp
    // 6- my port == dest port

    // 2-3 cycles:
    // 7- ip hdr_checksum is valid
    // 9- src ip is registered

    // until the very end:
    // 11- ip length matches until the tlast
    // 12- udp lenfth matched until the tlast

    typedef enum {
        IDLE
    } state_t;




endmodule
`default_nettype wire