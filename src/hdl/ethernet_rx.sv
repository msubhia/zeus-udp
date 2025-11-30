`default_nettype none

`include "zeus_rpc.svh"


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

// filter based on the src information (goes into the module)
// filter based ont the dest information (coming from config given to the ethernet rx module)

module ethernet_rx #(
    parameter int DATA_WIDTH = 512,
    parameter int KEEP_WIDTH = DATA_WIDTH / 8,
    parameter int KEY_WIDTH = 32 + 16,
    parameter int CONNECTION_ID_WIDTH = 32,
    parameter int MAC_ADDR_SIZE = 48
) (
    // ----------------------------------------------------------------
    // CMAC RX INFERFACE
    // ----------------------------------------------------------------
    input logic aclk,
    input logic aresetn,

    //////// RX ////////
    input  logic [DATA_WIDTH-1:0] s_axis_tdata,
    input  logic [KEEP_WIDTH-1:0] s_axis_tstrb,
    input  logic                  s_axis_tvalid,
    output logic                  s_axis_tready,
    input  logic                  s_axis_tlast,
    input  logic [           0:0] s_axis_tuser,

    // ----------------------------------------------------------------
    // OUTPUT
    // ----------------------------------------------------------------
    output logic [DATA_WIDTH-1:0] m_axis_tdata,
    output logic [KEEP_WIDTH-1:0] m_axis_tstrb,
    output logic [CONNECTION_ID_WIDTH - 1:0] m_axis_connection_id,
    output logic                             m_axis_length_valid, // if tlast is high and this is low, some bytes were dropped either from source or in logic itself (i.e. UDP packet length != transmitted length)
    output logic m_axis_tvalid,
    input logic m_axis_tready,
    output logic m_axis_tlast,
    output logic [0:0] m_axis_tuser,



    // ----------------------------------------------------------------
    // CTRL REGISTERS FOR CONNECTION MANAGER 
    // ----------------------------------------------------------------
    input  wire                  s02_axis_ctrl_valid,
    input  wire  [KEY_WIDTH-1:0] s02_axis_ctrl_key,
    input  wire                  s02_axis_ctrl_activate,
    output logic                 s02_axis_ctrl_ready,

    input  wire  m02_axis_ctrl_ready,  // assumes always 1
    output logic m02_axis_ctrl_valid,
    output logic m02_axis_ctrl_ack,
    output logic m02_axis_ctrl_full,

    // ----------------------------------------------------------------
    // DEST CONFIG
    // ----------------------------------------------------------------
    input connection_config_t my_config
);

  logic [MAC_ADDR_SIZE-1:0] my_mac_addr;
  logic [             31:0] my_ip_addr;
  logic [             15:0] my_port;
  assign my_mac_addr = my_config.mac_addr;
  assign my_ip_addr = my_config.ip_addr;
  assign my_port = my_config.port;


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
    IDLE,
    ACCUMULATING_PAYLOAD
  } state_t;


  state_t state, state_next;
  logic prev_tlast, is_packet_beginning;

  logic payload_valid, ppayload_valid, payload_tlast, ppayload_tlast;
  logic [KEEP_WIDTH-1:0] payload_tstrb, ppayload_tstrb;

  header_t header, current_header;
  logic [KEEP_WIDTH-1:0][7:0] payload, ppayload, s_axis_tdata_bytes;
  // should always be true
  logic connection_valid;
  logic [31:0] connection_id, connection_id_reg;
  logic connection_hit, connection_hit_reg;
  logic [15:0] udp_payload_length;
  logic [15:0] udp_payload_counter;
  logic connection_id_lookup_valid;

  connection_manager #(
      .KEY_WIDTH(32 + 16)  // ip addr + port
  ) conn_mgr (
      .clk(aclk),
      .rst(!aresetn),
      .s00_axis_fw_lookup_valid(connection_id_lookup_valid),
      .s00_axis_fw_lookup_ready(),  // ignored
      .s00_axis_fw_lookup_key({header.ip_hdr.src_ip, header.udp_hdr.src_port}),

      .m00_axis_fw_lookup_ready(1),
      .m00_axis_fw_lookup_valid(connection_valid),
      .m00_axis_fw_lookup_resp (connection_id_reg),
      .m00_axis_fw_lookup_hit  (connection_hit_reg),

      .s02_axis_ctrl_valid(s02_axis_ctrl_valid),
      .s02_axis_ctrl_key(s02_axis_ctrl_key),
      .s02_axis_ctrl_activate(s02_axis_ctrl_activate),
      .s02_axis_ctrl_ready(s02_axis_ctrl_ready),

      .m02_axis_ctrl_ready(m02_axis_ctrl_ready),
      .m02_axis_ctrl_valid(m02_axis_ctrl_valid),
      .m02_axis_ctrl_ack  (m02_axis_ctrl_ack),
      .m02_axis_ctrl_full (m02_axis_ctrl_full)
  );

  // only update on valid (would maintain previous value as long as we're not in the first cycle processing a header)
  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      connection_id  <= '0;
      connection_hit <= 1'b0;
    end else if (connection_valid) begin
      connection_id  <= connection_id_reg;
      connection_hit <= connection_hit_reg;
    end
  end

  assign s_axis_tdata_bytes = s_axis_tdata;
  assign s_axis_tready = 1'b1;
  assign header = header_t'(s_axis_tdata[DATA_WIDTH-1:0]);

  assign is_packet_beginning = s_axis_tvalid && prev_tlast;


  // pipelined signals:
  // connection id (automatically pipelined through the register above)
  // connection hit (automatically pipelined through the register above)
  // udp payload length
  // payload register data
  // payload output tstrb 

  logic payload_length_valid, ppayload_length_valid;

  assign payload_length_valid = prev_tlast && (udp_payload_counter == udp_payload_length);
  pipeline #(
      .DATA_WIDTH(1 + 1 + 1 + 512 + KEEP_WIDTH),
      .STAGES(3)
  ) payload_pipeline (
      .clk_in(aclk),
      .data({payload_length_valid, payload_tlast, payload_valid, payload, payload_tstrb}),
      .data_out({ppayload_length_valid, ppayload_tlast, ppayload_valid, ppayload, m_axis_tstrb})
  );


  // output channel assignments
  assign m_axis_tdata = ppayload;
  assign m_axis_connection_id = connection_id;
  assign m_axis_tvalid = ppayload_valid && connection_hit;
  assign m_axis_tlast = ppayload_tlast;
  assign m_axis_length_valid = ppayload_length_valid;



  // tstrb checks
  // header bytes = 52 bytes
  logic tstrb_prefix_ok;
  logic tstrb_suffix_ok;

  assign tstrb_prefix_ok = s_axis_tstrb[512/8-1: 512/8 -  HEADER_PAYLOAD_SIZE_BYTES] == {({HEADER_PAYLOAD_SIZE_BYTES{1'b1}})}; // want all header bytes to be present
  assign tstrb_suffix_ok = (s_axis_tstrb[KEEP_WIDTH-1: 52] != {KEEP_WIDTH-52{1'b0}}); // want at least one byte in the payload


  logic [47:0] dst_addr;
  logic [15:0] eth_type;
  logic [ 3:0] ip_version;
  logic [31:0] dst_ip;
  logic [ 7:0] ip_protocol;
  logic [15:0] dst_port;

  assign dst_addr = header.eth_hdr.dst_addr;
  assign eth_type = header.eth_hdr.eth_type;
  assign ip_version = header.ip_hdr.version;
  assign dst_ip = header.ip_hdr.dst_ip;
  assign ip_protocol = header.ip_hdr.protocol;
  assign dst_port = header.udp_hdr.dst_port;

  // header checks
  logic
      mac_ok,
      eth_ok,
      ipv4_ok,
      dst_ip_ok,
      proto_ok,
      port_ok,
      first_cycle_header_valid,
      first_cycle_payload_valid_next,
      single_cycle_udp_payload_len_ok;
  assign mac_ok = (dst_addr == my_mac_addr);
  assign eth_ok = (header.eth_hdr.eth_type == 16'h0800);
  assign ipv4_ok = (header.ip_hdr.version == 4);
  assign dst_ip_ok = (header.ip_hdr.dst_ip == my_ip_addr);
  assign proto_ok = (header.ip_hdr.protocol == IPPROTO_UDP);
  assign port_ok = (header.udp_hdr.dst_port == my_port);

  // -------------------------------------------
  // Combine everything into a single valid flag
  // -------------------------------------------

  assign first_cycle_header_valid =
	mac_ok & 
    tstrb_prefix_ok &
    tstrb_suffix_ok &
    mac_ok &
    eth_ok &
    ipv4_ok &
    dst_ip_ok &
    proto_ok &
    port_ok;

  localparam UDP_HDR_BYTES = 8;


  assign single_cycle_udp_payload_len_ok = header.udp_hdr.length - UDP_HDR_BYTES == $countones(
      tstrb_leftover
  );


  logic [HEADER_PAYLOAD_SIZE_BYTES-1:0][7:0] payload_leftover;
  logic [HEADER_PAYLOAD_SIZE_BYTES*8-1:0] payload_leftover_flat;
  logic payload_leftover_valid;
  assign payload_leftover_flat = header.payload_start;
  logic [HEADER_PAYLOAD_SIZE_BYTES-1:0] tstrb_leftover;
  logic [HEADER_PAYLOAD_SIZE_BYTES-1:0] tstrb_leftover_flat;
  assign tstrb_leftover_flat = s_axis_tstrb[HEADER_PAYLOAD_SIZE_BYTES-1:0];

  logic [15:0] payload_leftover_byte_count;
  assign payload_leftover_byte_count = $countones(tstrb_leftover);
  logic [15:0] total_active_bytes;
  assign total_active_bytes = $countones(s_axis_tstrb);


  logic transaction;
  assign transaction = s_axis_tvalid && s_axis_tready;


  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      state <= IDLE;
      prev_tlast <= 1'b1;
      connection_id_lookup_valid <= 0;
      tstrb_leftover <= '0;
      payload_leftover <= '0;
      payload_leftover_valid <= 1'b0;
      payload_tlast <= 0;
      payload_valid <= 0;
      payload_tstrb <= '0;
      payload <= 0;
      udp_payload_length <= 16'b0;
      udp_payload_counter <= 16'b0;
    end else begin

      if (transaction) begin
        prev_tlast <= s_axis_tlast;
      end

      case (state)
        IDLE: begin
          if (transaction && is_packet_beginning && first_cycle_header_valid) begin
            // check the validity of the header before moving to accumulating


            // if the header is initially valid
            // load relevant header information and initiate a connection id lookup

            // if valid, start looking up the connection id
            connection_id_lookup_valid <= 1'b1;

            // if accumulating, save the expected and current payload count from header								 
            udp_payload_length <= header.udp_hdr.length - UDP_HDR_BYTES;
            udp_payload_counter <= total_active_bytes - HEADER_SIZE_BYTES;

            state <= ACCUMULATING_PAYLOAD;

            for (int i = 0; i < HEADER_PAYLOAD_SIZE_BYTES; i++) begin
              payload_leftover[i] <= payload_leftover_flat[(i+1)*8-1-:8];
              tstrb_leftover[i]   <= tstrb_leftover_flat[i];
            end
            payload_leftover_valid <= 1'b1;

            if (payload_leftover_valid) begin
              // leftover always goes in the first bytes of the actual payload
              for (int i = 0; i < HEADER_PAYLOAD_SIZE_BYTES; i++) begin
                payload[i] <= payload_leftover[i];
                payload_tstrb[i] <= tstrb_leftover[i];
              end
              for (int i = 0; i < KEEP_WIDTH - HEADER_PAYLOAD_SIZE_BYTES; i++) begin
                payload_tstrb[HEADER_PAYLOAD_SIZE_BYTES+i] <= 0;
              end
              payload_valid <= 1'b1;
              payload_tlast <= prev_tlast;
            end else begin
              payload_valid <= 0;
            end
          end else begin
            if (payload_leftover_valid) begin
              // leftover always goes in the first bytes of the actual payload
              for (int i = 0; i < HEADER_PAYLOAD_SIZE_BYTES; i++) begin
                payload[i] <= payload_leftover[i];
                payload_tstrb[i] <= tstrb_leftover[i];
              end
              for (int i = 0; i < KEEP_WIDTH - HEADER_PAYLOAD_SIZE_BYTES; i++) begin
                payload_tstrb[HEADER_PAYLOAD_SIZE_BYTES+i] <= 0;
              end
              payload_valid <= 1'b1;
              payload_tlast <= prev_tlast;
              payload_leftover_valid <= 1'b0;
            end else begin
              payload_valid <= 0;
            end
          end
          // otherwise we stay in idle 
        end

        ACCUMULATING_PAYLOAD: begin
          connection_id_lookup_valid <= 1'b0;
          if (transaction) begin
            // if (s_axis_tlast) begin
            //   state <= IDLE;
            // end
            // // put the rest of the payload in payload leftover
            // udp_payload_counter <= udp_payload_counter + total_active_bytes;
            // if (payload_leftover_valid) begin
            //   payload_leftover_valid <= 1'b0;  // will be flushed by next cycle
            // end

            if (is_packet_beginning && first_cycle_header_valid) begin
              // update the leftover registers
              for (int i = 0; i < HEADER_PAYLOAD_SIZE_BYTES; i++) begin
                payload_leftover[i] <= payload_leftover_flat[(i+1)*8-1-:8];
                tstrb_leftover[i]   <= tstrb_leftover_flat[i];
              end
              payload_leftover_valid <= 1'b1;

              // TODO(menaf): this causes a length comparison bug... need to fix this with some delayed assignment of UDP_PAYLOAD_LENGTH and UDP_PAYLOAD_COUNTER

              // if accumulating, save the expected and current payload count from header								 
              udp_payload_length <= header.udp_hdr.length - UDP_HDR_BYTES;
              udp_payload_counter <= total_active_bytes - HEADER_SIZE_BYTES;

              if (payload_leftover_valid) begin
                // start a new packet while still having leftover from previous one
                for (int i = 0; i < HEADER_PAYLOAD_SIZE_BYTES; i++) begin
                  payload[i] <= payload_leftover[i];
                  payload_tstrb[i] <= tstrb_leftover[i];
                end
                for (int i = 0; i < KEEP_WIDTH - HEADER_PAYLOAD_SIZE_BYTES; i++) begin
                  payload_tstrb[HEADER_PAYLOAD_SIZE_BYTES+i] <= 0;
                end
                payload_valid <= 1'b1;
                payload_tlast <= prev_tlast;
              end

            end else begin
              if (payload_leftover_valid) begin
                // combine with leftover as msb and fill the rest from the current transaction

                for (int i = 0; i < HEADER_PAYLOAD_SIZE_BYTES; i++) begin
                  payload[KEEP_WIDTH-1-i] <= payload_leftover[HEADER_PAYLOAD_SIZE_BYTES-1-i];
                  payload_tstrb[KEEP_WIDTH-1-i] <= tstrb_leftover[HEADER_PAYLOAD_SIZE_BYTES-1-i];
                end

                for (int i = 0; i < KEEP_WIDTH - HEADER_PAYLOAD_SIZE_BYTES; i++) begin
                  payload[KEEP_WIDTH-1 - (HEADER_PAYLOAD_SIZE_BYTES+i)] <= s_axis_tdata_bytes[KEEP_WIDTH-1-i];
                  payload_tstrb[KEEP_WIDTH-1 - (HEADER_PAYLOAD_SIZE_BYTES+i)] <= s_axis_tstrb[KEEP_WIDTH-1-i];
                end

                for (int i = 0; i < HEADER_PAYLOAD_SIZE_BYTES; i++) begin
                  payload_leftover[i] <= payload_leftover_flat[(i+1)*8-1-:8];
                  tstrb_leftover[i]   <= tstrb_leftover_flat[i];
                end

                payload_tlast <= prev_tlast;

                // udp payload counter update
                udp_payload_counter <= udp_payload_counter + total_active_bytes;

              end else begin
                // passthrough behavior
                for (int i = 0; i < KEEP_WIDTH; i++) begin
                  payload[i] <= s_axis_tdata_bytes[i];
                  payload_tstrb[i] <= s_axis_tstrb[i];
                end

                payload_tlast <= s_axis_tlast;
              end
              udp_payload_counter <= udp_payload_counter + total_active_bytes;
              payload_valid <= 1'b1;

              if (s_axis_tlast) begin
                state <= IDLE;
              end
            end

          end else begin
            // flush the payload leftover if no new data
            if (payload_leftover_valid) begin
              for (int i = 0; i < HEADER_PAYLOAD_SIZE_BYTES; i++) begin
                payload[i] <= payload_leftover[i];
                payload_tstrb[i] <= tstrb_leftover[i];
              end

              for (int i = 0; i < KEEP_WIDTH - HEADER_PAYLOAD_SIZE_BYTES; i++) begin
                payload_tstrb[HEADER_PAYLOAD_SIZE_BYTES+i] <= 0;
              end

              payload_valid <= 1'b1;
              payload_tlast <= prev_tlast;
              // udp payload counter does not change since 
              // we're not getting any new data, so we can flush the leftover
              payload_leftover_valid <= 1'b0;
            end else begin
              payload_valid <= 0;
            end
          end
        end
      endcase
    end
  end

endmodule
`default_nettype wire
