`default_nettype none
`timescale 1ns/1ps

// ============================================================================
// Ethernet TX IP
// ============================================================================
//
// Author:          M.Subhi Abordan (msubhi_a@mit.edu)
// Last Modified:   Nov 28, 2025
//
// ============================================================================
// END
// ============================================================================


function automatic logic [8*NUM_BYTES-1:0] swap_bytes #(int NUM_BYTES = 4)(input logic [8*NUM_BYTES-1:0] din);
    for (int i = 0; i < NUM_BYTES; i++) begin
        swap_bytes[(i*8+7): (i*8)] = din[(NUM_BYTES-1-i)*8 +7: (NUM_BYTES-1-i)*8];
    end
endfunction

// so far this assumes down stream is always ready 

module ethernet_header_prepend #(
    parameter  int DATA_WIDTH       = 512,              // minimum 128
    localparam int KEEP_WIDTH       = DATA_WIDTH/8,
) (
    // ----------------------------------------------------------------
    // Up-stream
    // ----------------------------------------------------------------
    input wire                      s00_axis_aclk, 
    input wire                      s00_axis_aresetn,
    input wire                      s00_axis_tvalid,
    input wire [DATA_WIDTH-1:0]     s00_axis_tdata,
    input wire [KEEP_WIDTH-1:0]     s00_axis_tkeep,
    input wire                      s00_axis_tlast,
    input wire [1:0]                s00_axis_tuser,
    output logic                    s00_axis_tready,

    // ----------------------------------------------------------------
    // down-stream
    // ----------------------------------------------------------------
    output logic                    m00_axis_tvalid,
    output logic [DATA_WIDTH-1:0]   m00_axis_tdata,
    output logic [KEEP_WIDTH-1:0]   m00_axis_tkeep,
    output logic                    m00_axis_tlast,
    output logic [1:0]              m00_axis_tuser,
    input wire                      m00_axis_tready,

    // ----------------------------------------------------------------
    // connection configurations
    // ----------------------------------------------------------------
    input wire [47:0]     src_mac_addr_in,
    input wire [47:0]     dst_mac_addr_in,
    input wire [47:0]     eth_type_in
);
    // Ethernet header is 14 bytes:
    //  - 6-byte destination MAC
    //  - 6-byte source MAC
    //  - 2-byte EtherType field
    // total is 14 * 8 = 112 bits 
    localparam ETH_HEADER_WIDTH = 112;
    localparam ETH_HEADER_BYTES = ETH_HEADER_WIDTH/8;
    // Ethernet is Big Endianess
    logic [111:0] eth_header_in;
    always_comb begin
        eth_header_in[47:0]    = swap_bytes #(.NUM_BYTES(6))(dst_mac_addr_in);
        eth_header_in[95:48]   = swap_bytes #(.NUM_BYTES(6))(src_mac_addr_in);
        eth_header_in[111:96]  = swap_bytes #(.NUM_BYTES(2))(eth_type_in);
    end

    typedef enum logic[1:0] {STATE_IDLE, STATE_TRANSMIT, STATE_TAIL} state_t;
    state_t state;

    logic [ETH_HEADER_WIDTH-1:0] internal_shift_reg;

    logic [1:0]  m00_axis_tuser_tail;
    logic [KEEP_WIDTH-1:0] m00_axis_tkeep_tail;

    always_ff @(posedge s00_axis_aclk) begin

        if (!s00_axis_aresetn) begin
            state               <= STATE_IDLE;
            m00_axis_tvalid     <= 'b0;
            m00_axis_tdata      <= 'b0;
            internal_shift_reg  <= 'b0;
            m00_axis_tkeep_tail <= 'b0;
            m00_axis_tuser_tail <= 'b0;

            s00_axis_tready     <= 1'b0;

        end else begin
            s00_axis_tready     <= 1'b1;

            case (state)

                STATE_IDLE: begin

                    if (s00_axis_tvalid && s00_axis_tready) begin

                        m00_axis_tvalid     <= s00_axis_tvalid;
                        m00_axis_tuser      <= s00_axis_tuser;

                        m00_axis_tdata      <= {s00_axis_tdata[LEFT_WIDTH-1:0], eth_header_in};
                        internal_shift_reg  <= s00_axis_tdata[DATA_WIDTH-1 : DATA_WIDTH-ETH_HEADER_WIDTH];

                        if (s00_axis_tlast) begin
                            if (s00_axis_tkeep <= {(ETH_HEADER_BYTES){1'b0}, (KEEP_WIDTH-ETH_HEADER_BYTES){1'b1}}) begin
                                // packet with it's content can fit in one transaction
                                m00_axis_tkeep <=  (s00_axis_tkeep<<(ETH_HEADER_BYTES)) | {(KEEP_WIDTH-ETH_HEADER_BYTES){1'b0}, (ETH_HEADER_BYTES){1'b1}};
                                m00_axis_tlast <= 1'b1;
                                state <= IDLE;
                            end else begin
                                // neeed an extra cycle
                                m00_axis_tkeep <= {KEEP_WIDTH{1'b1}};
                                m00_axis_tlast <= 1'b0;
                                state          <= TAIL;
                                s00_axis_tready <= 1'b0;
                                m00_axis_tkeep_tail <= (s00_axis_tkeep >> 50);
                                m00_axis_tuser_tail <= s00_axis_tuser;
                            end
                        end else begin
                            state          <= STATE_TRANSMIT;
                            m00_axis_tkeep <= {KEEP_WIDTH{1'b1}};
                            m00_axis_tlast <= 1'b0;
                        end
                    end

                end

                STATE_TRANSMIT: begin
                    m00_axis_tuser      <= s00_axis_tuser;
                    m00_axis_tvalid     <= s00_axis_tvalid;

                    if (s00_axis_tvalid) begin
                        m00_axis_tdata      <= {s00_axis_tdata[LEFT_WIDTH-1:0], internal_shift_reg};
                        internal_shift_reg  <= s00_axis_tdata[DATA_WIDTH-1 : DATA_WIDTH-ETH_HEADER_WIDTH];

                        if (s00_axis_tlast) begin
                            if (s00_axis_tkeep <= {(ETH_HEADER_BYTES){1'b0}, (KEEP_WIDTH-ETH_HEADER_BYTES){1'b1}}) begin
                                // packet with it's content can fit in one transaction
                                m00_axis_tkeep <=  (s00_axis_tkeep<<(ETH_HEADER_BYTES)) | {(KEEP_WIDTH-ETH_HEADER_BYTES){1'b0}, (ETH_HEADER_BYTES){1'b1}};
                                m00_axis_tlast <= 1'b1;
                                state          <= IDLE;
                            end else begin
                                // neeed an extra cycle
                                m00_axis_tkeep <= {KEEP_WIDTH{1'b1}};
                                m00_axis_tlast <= 1'b0;
                                state          <= TAIL;
                                s00_axis_tready <= 1'b0;
                                m00_axis_tuser_tail <= s00_axis_tuser;
                                m00_axis_tkeep_tail <= (s00_axis_tkeep >> 50);
                            end
                        end else begin
                            state          <= STATE_TRANSMIT;
                            m00_axis_tkeep <= {KEEP_WIDTH{1'b1}};
                            m00_axis_tlast <= 1'b0;
                        end
                    end
                end

                STATE_TAIL: begin
                    m00_axis_tvalid     <= 1'b1;
                    m00_axis_tdata      <= {'b0, internal_shift_reg};
                    m00_axis_tkeep      <= m00_axis_tkeep_tail;
                    m00_axis_tuser      <= m00_axis_tuser_tail;
                    m00_axis_tlast      <= 1'b1;
                    state               <= STATE_IDLE;
                end

            endcase

        end

    end
    
endmodule