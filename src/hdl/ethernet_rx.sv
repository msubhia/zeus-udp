`default_nettype none `timescale 1ns / 1ps

`include "zeus_rpc.svh"

// ============================================================================
// Ethernet RX IP
// ============================================================================
//
// Authors:          M.Subhi Abordan (msubhi_a@mit.edu)
//                   Mena Filfil     (menaf@mit.edu)
// Last Modified:    Dec 1, 2025
//
// ============================================================================
// END
// ============================================================================

module ethernet_rx #(
    parameter int DATA_WIDTH         = 512,
    parameter int CONN_ID_WIDTH      = 18,
    parameter int IP_UDP_DSCP        = 0,
    parameter int IP_UDP_ENC         = 0,
    parameter int IP_UDP_IDEN        = 0,
    parameter int IP_UDP_FLAGS       = 0,
    parameter int IP_UDP_FRAG_OFFSET = 0,
    parameter int IP_UDP_TTL         = 64,

    localparam int IP_ADDR_WIDTH = 32,
    localparam int MAC_ADDR_WIDTH = 48,
    localparam int UDP_PORT_WIDTH = 16,
    localparam int CONNECTION_METADATA_WIDTH = IP_ADDR_WIDTH + MAC_ADDR_WIDTH + UDP_PORT_WIDTH + 8,
    localparam int IP_PACKET_LENGTH_WIDTH = 16,
    localparam int ETH_HEADER_BYTES = 14,
    localparam int IP_HEADER_BYTES = 20,
    localparam int UDP_HEADER_BYTES = 8,
    localparam int TOTAL_HEADERS_BYTES = ETH_HEADER_BYTES + IP_HEADER_BYTES + UDP_HEADER_BYTES,
    localparam int TOTAL_HEADERS_BITS = TOTAL_HEADERS_BYTES * 8
) (
    // ----------------------------------------------------------------
    // CONTROL AND STATUS
    // ----------------------------------------------------------------
    input wire rx_axis_aclk,
    input wire rx_axis_aresetn,
    // input wire                          rx_engine_bypass,
    // input wire                          rx_engine_enable,

    // ----------------------------------------------------------------
    // CONNECTION CONFIGURATION
    // ----------------------------------------------------------------
    input wire [ IP_ADDR_WIDTH-1:0] my_config_dst_ipAddr,
    input wire [MAC_ADDR_WIDTH-1:0] my_config_dst_macAddr,
    input wire [UDP_PORT_WIDTH-1:0] my_config_dst_udpPort,
    input wire [MAC_ADDR_WIDTH-1:0] my_config_src_macAddr,

    // ----------------------------------------------------------------
    // CMAC RX
    // ----------------------------------------------------------------
    output logic                    cmac_rx_axis_tready,
    input  wire  [  DATA_WIDTH-1:0] cmac_rx_axis_tdata,
    input  wire  [DATA_WIDTH/8-1:0] cmac_rx_axis_tkeep,
    input  wire                     cmac_rx_axis_tvalid,
    input  wire                     cmac_rx_axis_tlast,

    // ----------------------------------------------------------------
    // USER OUTPUT
    // ----------------------------------------------------------------
    input  wire                     udp_rx_axis_tready,
    output logic [  DATA_WIDTH-1:0] udp_rx_axis_tdata,
    output logic [DATA_WIDTH/8-1:0] udp_rx_axis_tkeep,
    output logic                    udp_rx_axis_tvalid,
    output logic                    udp_rx_axis_tlast,

    // ----------------------------------------------------------------
    // CONNECTION MANAGER ID LOOKUP
    // ----------------------------------------------------------------
    input  wire                       m01_axis_fw_lookup_ready,
    output logic                      m01_axis_fw_lookup_valid,
    output logic [ IP_ADDR_WIDTH-1:0] m01_axis_fw_lookup_ipAddr,
    output logic [UDP_PORT_WIDTH-1:0] m01_axis_fw_lookup_udpPort,

    output logic                     s01_axis_fw_lookup_ready,
    input  wire                      s01_axis_fw_lookup_valid,
    input  wire                      s01_axis_fw_lookup_hit,
    input  wire  [CONN_ID_WIDTH-1:0] s01_axis_fw_lookup_connectionId
);

  // ----------------------------------------------------------------------------------------------
  // FIFOs
  // ----------------------------------------------------------------------------------------------

  // As packets are entering we count their length on the fly and forward them to a fifo, once an
  // entire packet has entered we send its length in another fifo a long with the fetched connection
  // meta data
  // a consuming fsm on the other side of the fifo starts when all relavent metadata of a packet in
  // the fifo arrives, and it constructs the header and prepend it
  logic                           payload_fifo_tready;
  logic                           payload_fifo_tlast;
  logic                           payload_fifo_tvalid;
  logic [       DATA_WIDTH  -1:0] payload_fifo_tdata;
  logic [       DATA_WIDTH/8-1:0] payload_fifo_tkeep;

  logic                           connection_fifo_tready;
  logic                           connection_fifo_tlast;
  logic                           connection_fifo_tvalid;
  logic [(CONN_ID_WIDTH-1 + 6):0] connection_fifo_tdata;

  logic                           length_check_fifo_out_tready;
  logic                           length_check_fifo_out_tlast;
  logic                           length_check_fifo_out_tvalid;
  logic [                    7:0] length_check_fifo_out_tdata;

  logic                           length_check_fifo_tready;
  logic                           length_check_fifo_tvalid;
  logic                           length_check_fifo_tdata;

  logic                           header_valid_eager;

  logic cmac_rx_axis_treadyp, cmac_rx_axis_tlastp, cmac_rx_axis_tvalidp;
  logic [  DATA_WIDTH-1:0] cmac_rx_axis_tdatap;
  logic [DATA_WIDTH/8-1:0] cmac_rx_axis_tkeepp;

  always_ff @(posedge rx_axis_aclk) begin
    cmac_rx_axis_tready  <= cmac_rx_axis_treadyp;
    cmac_rx_axis_tdatap  <= cmac_rx_axis_tdata;
    cmac_rx_axis_tkeepp  <= cmac_rx_axis_tkeep;
    cmac_rx_axis_tlastp  <= cmac_rx_axis_tlast;
    cmac_rx_axis_tvalidp <= cmac_rx_axis_tvalid && header_valid_eager;
  end


  fifo_axis_wrapper #(
      .FIFO_DEPTH (64),
      .TDATA_WIDTH(DATA_WIDTH)
  ) packet_transaction_fifo (
      // input from CMAC RX
      .s_aclk(rx_axis_aclk),
      .s_aresetn(rx_axis_aresetn),
      .s_axis_tdata(cmac_rx_axis_tdatap),
      .s_axis_tkeep(cmac_rx_axis_tkeepp),
      .s_axis_tlast(cmac_rx_axis_tlastp),
      .s_axis_tvalid(cmac_rx_axis_tvalidp  /*& tx_engine_enable & (!tx_engine_bypass)*/),
      .s_axis_tready(cmac_rx_axis_treadyp),  // should always be ready

      // output to payload processing signals
      .m_aclk(rx_axis_aclk),
      .m_axis_tready(payload_fifo_tready),
      .m_axis_tlast(payload_fifo_tlast),
      .m_axis_tvalid(payload_fifo_tvalid),
      .m_axis_tdata(payload_fifo_tdata),
      .m_axis_tkeep(payload_fifo_tkeep)
  );

  fifo_axis_wrapper #(
      .FIFO_DEPTH (64),
      .TDATA_WIDTH(CONN_ID_WIDTH + 1 + 5)
  ) connection_id_fifo (
      // connection ids coming from connection manager
      .s_aclk(rx_axis_aclk),
      .s_aresetn(rx_axis_aresetn),
      .s_axis_tdata({5'b0, s01_axis_fw_lookup_hit, s01_axis_fw_lookup_connectionId}),
      .s_axis_tkeep({((CONN_ID_WIDTH + 6) / 8) {1'b1}}),
      .s_axis_tlast(1'b1),
      .s_axis_tvalid(s01_axis_fw_lookup_valid  /*& tx_engine_enable & (!tx_engine_bypass)*/),
      .s_axis_tready(s01_axis_fw_lookup_ready),  // should always be ready

      // connection ids going to the payload processor
      .m_aclk(rx_axis_aclk),
      .m_axis_tready(connection_fifo_tready),
      .m_axis_tlast(connection_fifo_tlast),
      .m_axis_tvalid(connection_fifo_tvalid),
      .m_axis_tdata(connection_fifo_tdata),
      .m_axis_tkeep()
  );

  fifo_axis_wrapper #(
      .FIFO_DEPTH (64),
      .TDATA_WIDTH(8)
  ) length_check_fifo (
      // length checking on running counter of packet lengths
      .s_aclk(rx_axis_aclk),
      .s_aresetn(rx_axis_aresetn),
      .s_axis_tdata({7'b0, length_check_fifo_tdata}),
      .s_axis_tkeep(1'b1),
      .s_axis_tlast(1'b1),
      .s_axis_tvalid(length_check_fifo_tvalid  /*& tx_engine_enable & (!tx_engine_bypass)*/),
      .s_axis_tready(length_check_fifo_tready),  // should always be ready

      // length check output to payload processor
      .m_aclk(rx_axis_aclk),
      .m_axis_tready(length_check_fifo_out_tready),
      .m_axis_tlast(length_check_fifo_out_tlast),
      .m_axis_tvalid(length_check_fifo_out_tvalid),
      .m_axis_tdata(length_check_fifo_out_tdata),
      .m_axis_tkeep()
  );

  // ----------------------------------------------------------------------------------------------
  // Transaction Sequencing Metadata
  // ----------------------------------------------------------------------------------------------
  logic is_first_transaction;

  always_ff @(posedge rx_axis_aclk) begin
    if (!rx_axis_aresetn) is_first_transaction <= 1;
    else if (cmac_rx_axis_tvalid && cmac_rx_axis_tready) begin
      if (cmac_rx_axis_tlast) is_first_transaction <= 1;
      else is_first_transaction <= 0;
    end
  end

  // ----------------------------------------------------------------------------------------------
  // Header Parsing and Validation
  // ----------------------------------------------------------------------------------------------
  header_t full_header;
  // little endian to big endian conversion to correctly parse packet headers from CMAC in network order
  logic [DATA_WIDTH-1:0] cmac_rx_axis_tdata_be;
  logic
      header_valid,
      dst_mac_valid,
      src_mac_valid,
      dst_ip_valid,
      dst_port_valid,
      eth_type_valid,
      ip_proto_valid,
      ip_version_valid;
  generate
    // split into bytes and reverse order byte
    for (genvar i = 0; i < DATA_WIDTH / 8; i++) begin : be_gen
      assign cmac_rx_axis_tdata_be[i*8+:8] = cmac_rx_axis_tdata[(DATA_WIDTH-8)-i*8+:8];
    end
  endgenerate

  assign full_header = cmac_rx_axis_tdata_be;

  logic [15:0] udp_rx_axis_length;
  assign udp_rx_axis_length = full_header.udp_hdr.length - (UDP_HEADER_BYTES);

  logic [47:0] dst_mac;
  logic [47:0] src_mac;
  logic [15:0] eth_type;
  logic [ 3:0] ip_version;
  logic [ 3:0] ip_header_length;
  logic [31:0] src_ip;
  logic [31:0] dst_ip;
  logic [15:0] udp_src_port;
  logic [15:0] udp_dst_port;
  logic [ 7:0] ip_proto;

  assign dst_mac = full_header.eth_hdr.dst_mac;
  assign src_mac = full_header.eth_hdr.src_mac;
  assign eth_type = full_header.eth_hdr.eth_type;
  assign ip_version = full_header.ip_hdr.version;
  assign ip_header_length = full_header.ip_hdr.header_length;
  assign src_ip = full_header.ip_hdr.src_ip;
  assign dst_ip = full_header.ip_hdr.dst_ip;
  assign ip_proto = full_header.ip_hdr.protocol;
  assign udp_src_port = full_header.udp_hdr.src_port;
  assign udp_dst_port = full_header.udp_hdr.dst_port;

  // ----------------------------------------------------------------------------------------------

  assign dst_mac_valid = (full_header.eth_hdr.dst_mac == my_config_dst_macAddr);
  assign src_mac_valid = (full_header.eth_hdr.src_mac == my_config_src_macAddr);
  assign eth_type_valid = (full_header.eth_hdr.eth_type == 16'h0800);  // IPv4
  assign ip_proto_valid = (full_header.ip_hdr.protocol == 8'h11);  // UDP
  assign dst_ip_valid = (full_header.ip_hdr.dst_ip == my_config_dst_ipAddr);
  assign dst_port_valid = (full_header.udp_hdr.dst_port == my_config_dst_udpPort);
  assign ip_version_valid = (full_header.ip_hdr.version == IP_VERSION_IPV4);
  assign header_valid = dst_mac_valid & src_mac_valid & eth_type_valid & ip_proto_valid & dst_ip_valid & dst_port_valid & ip_version_valid;

  // ----------------------------------------------------------------------------------------------
  // Length Sum and Expected Capture and Check
  // ----------------------------------------------------------------------------------------------
  // holds a valid value from the first transaction through the end of the packet (including the tlast cycle)
  // this fsm should only initiate on the first transaction of a packet with a valid
  logic [IP_PACKET_LENGTH_WIDTH-1:0] expected_udp_length;
  logic                              expected_udp_length_valid;
  logic [IP_PACKET_LENGTH_WIDTH-1:0] expected_udp_length_post;
  logic                              expected_udp_length_post_valid;
  logic [IP_PACKET_LENGTH_WIDTH-1:0] current_udp_length;
  logic [IP_PACKET_LENGTH_WIDTH-1:0] current_udp_length_post;
  logic                              current_udp_length_post_valid;


  logic                              header_valid_reg;
  // FSM for header validity
  // header_valid_reg rises on the first transaction if the header is valid
  // stays high until and including the tlast of the stream of incoming cmac_rx transactions
  // resets to 0 on rx_axis_aresetn
  always_ff @(posedge rx_axis_aclk) begin
    if (!rx_axis_aresetn) begin
      header_valid_reg <= 1'b0;
    end else begin
      if (cmac_rx_axis_tvalid && cmac_rx_axis_tready) begin
        if (is_first_transaction) begin
          header_valid_reg <= header_valid;
        end else if (cmac_rx_axis_tlast) begin
          header_valid_reg <= 1'b0;
        end
      end
    end
  end
  assign header_valid_eager = is_first_transaction ? header_valid : header_valid_reg;

  always_ff @(posedge rx_axis_aclk) begin
    if (!rx_axis_aresetn) begin
      expected_udp_length            <= 'b0;
      expected_udp_length_valid      <= 1'b0;
      expected_udp_length_post       <= 'b0;
      expected_udp_length_post_valid <= 1'b0;
    end else begin
      if (cmac_rx_axis_tvalid && cmac_rx_axis_tready) begin
        if (is_first_transaction) begin
          if (cmac_rx_axis_tlast) begin
            expected_udp_length            <= 'b0;
            expected_udp_length_post       <= udp_rx_axis_length;
            expected_udp_length_post_valid <= is_first_transaction && header_valid;
          end else begin
            expected_udp_length            <= udp_rx_axis_length;
            expected_udp_length_valid      <= is_first_transaction && header_valid;
            expected_udp_length_post       <= 'b0;
            expected_udp_length_post_valid <= 1'b0;
          end
        end else if (cmac_rx_axis_tlast) begin
          expected_udp_length_post       <= expected_udp_length;
          expected_udp_length_post_valid <= expected_udp_length_valid;
        end else begin
          expected_udp_length_post       <= 'b0;
          expected_udp_length_post_valid <= 1'b0;
        end
      end else begin
        expected_udp_length_post_valid <= 1'b0;
      end
    end
  end

  always_ff @(posedge rx_axis_aclk) begin

    if (!rx_axis_aresetn) begin
      current_udp_length            <= 'b0;
      current_udp_length_post       <= 'b0;
      current_udp_length_post_valid <= 1'b0;
    end else begin

      if (cmac_rx_axis_tvalid && cmac_rx_axis_tready) begin
        if (cmac_rx_axis_tlast) begin
          current_udp_length <= 'b0;
          current_udp_length_post <= current_udp_length + $countones(
              cmac_rx_axis_tkeep
          ) - (TOTAL_HEADERS_BYTES);
          current_udp_length_post_valid <= 1'b1;
        end else begin
          current_udp_length            <= current_udp_length + $countones(cmac_rx_axis_tkeep);
          current_udp_length_post       <= 'b0;
          current_udp_length_post_valid <= 1'b0;
        end
      end else begin
        current_udp_length_post_valid <= 1'b0;
      end

    end
  end

  logic length_check_passed;
  assign length_check_passed = (expected_udp_length_post_valid && current_udp_length_post_valid) ?
							  (expected_udp_length_post == current_udp_length_post) : 1'b0;
  assign length_check_fifo_tdata = length_check_passed;
  assign length_check_fifo_tvalid = expected_udp_length_post_valid && current_udp_length_post_valid;

  // ----------------------------------------------------------------------------------------------
  // ConnectionId Fetch
  // ----------------------------------------------------------------------------------------------

  assign m01_axis_fw_lookup_valid = cmac_rx_axis_tvalid && cmac_rx_axis_tready && is_first_transaction && header_valid;
  assign m01_axis_fw_lookup_ipAddr = full_header.ip_hdr.src_ip;
  assign m01_axis_fw_lookup_udpPort = full_header.udp_hdr.src_port;


  // ==============================================================================================
  // Payload Generation
  // ==============================================================================================

  rx_payload_constructor #(
      .DATA_WIDTH(DATA_WIDTH),
      .CONN_ID_WIDTH(CONN_ID_WIDTH),
      .IP_UDP_DSCP(IP_UDP_DSCP),
      .IP_UDP_ENC(IP_UDP_ENC),
      .IP_UDP_IDEN(IP_UDP_IDEN),
      .IP_UDP_FLAGS(IP_UDP_FLAGS),
      .IP_UDP_FRAG_OFFSET(IP_UDP_FRAG_OFFSET),
      .IP_UDP_TTL(IP_UDP_TTL)
  ) rx_payload_constructor_unit (
      .rx_axis_aclk(rx_axis_aclk),
      .rx_axis_aresetn(rx_axis_aresetn),

      .payload_fifo_tlast (payload_fifo_tlast),
      .payload_fifo_tvalid(payload_fifo_tvalid),
      .payload_fifo_tdata (payload_fifo_tdata),
      .payload_fifo_tkeep (payload_fifo_tkeep),
      .payload_fifo_tready(payload_fifo_tready),

      .connection_fifo_tlast (connection_fifo_tlast),
      .connection_fifo_tvalid(connection_fifo_tvalid),
      .connection_fifo_tdata (connection_fifo_tdata),
      .connection_fifo_tready(connection_fifo_tready),

      .length_check_fifo_out_tlast (length_check_fifo_out_tlast),
      .length_check_fifo_out_tvalid(length_check_fifo_out_tvalid),
      .length_check_fifo_out_tdata (length_check_fifo_out_tdata),
      .length_check_fifo_out_tready(length_check_fifo_out_tready),

      .payload_out_tready(udp_rx_axis_tready_prev),
      .payload_out_tdata (udp_rx_axis_tdata_prev),
      .payload_out_tkeep (udp_rx_axis_tkeep_prev),
      .payload_out_tvalid(udp_rx_axis_tvalid_prev),
      .payload_out_tlast (udp_rx_axis_tlast_prev)
  );

  // ==============================================================================================
  // FSM for removing empty transactions at the end of a packet
  // ==============================================================================================
  logic [  DATA_WIDTH-1:0] udp_rx_axis_tdata_reg;
  logic [DATA_WIDTH/8-1:0] udp_rx_axis_tkeep_reg;
  logic                    udp_rx_axis_tlast_reg;
  logic                    udp_rx_axis_tvalid_reg;
  logic                    udp_rx_axis_tready_reg;

  logic [  DATA_WIDTH-1:0] udp_rx_axis_tdata_prev;
  logic [DATA_WIDTH/8-1:0] udp_rx_axis_tkeep_prev;
  logic                    udp_rx_axis_tvalid_prev;
  logic                    udp_rx_axis_tlast_prev;
  logic                    udp_rx_axis_tready_prev;


  always_ff @(posedge rx_axis_aclk) begin
    if (!rx_axis_aresetn) begin
      udp_rx_axis_tdata  <= '0;
      udp_rx_axis_tkeep  <= '0;
      udp_rx_axis_tvalid <= 1'b0;
      udp_rx_axis_tlast  <= 1'b0;
    end else begin
      udp_rx_axis_tdata_reg <= udp_rx_axis_tdata_prev;
      udp_rx_axis_tkeep_reg <= udp_rx_axis_tkeep_prev;
      udp_rx_axis_tlast_reg <= udp_rx_axis_tlast_prev;
      udp_rx_axis_tvalid_reg <= udp_rx_axis_tvalid_prev;
      udp_rx_axis_tready_prev <= udp_rx_axis_tready_reg;

      // reg to out
      udp_rx_axis_tdata <= udp_rx_axis_tdata_reg;
      udp_rx_axis_tkeep <= udp_rx_axis_tkeep_reg;
      udp_rx_axis_tlast <= udp_rx_axis_tlast_reg || (udp_rx_axis_tlast_prev && (udp_rx_axis_tkeep_prev == '0));
      udp_rx_axis_tvalid <= udp_rx_axis_tvalid_reg && (udp_rx_axis_tkeep_reg != '0);
      udp_rx_axis_tready_reg <= udp_rx_axis_tready;
    end
  end



endmodule
`default_nettype wire
