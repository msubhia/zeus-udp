`default_nettype none `timescale 1ns / 1ps
`include "zeus_rpc.svh"

// ----------------------------------------------------------------
// RX Payload Constructor
// ----------------------------------------------------------------

module clz_tree #(
    parameter N = 32
) (
    input wire [N-1:0] in,
    output logic [$clog2(N):0] clz
);

  generate
    if (N == 1) begin : base
      assign clz = (in[0] == 1'b1) ? 0 : 1;

    end else begin : rec
      localparam HALF = N / 2;

      // Split
      logic [HALF-1:0] hi;
      logic [HALF-1:0] lo;

      assign hi = in[N-1:HALF];
      assign lo = in[HALF-1:0];

      // Sub-results
      logic [$clog2(HALF):0] clz_hi;
      logic [$clog2(HALF):0] clz_lo;

      // Recursive instantiations
      clz_tree #(HALF) upper (
          .in (hi),
          .clz(clz_hi)
      );

      clz_tree #(HALF) lower (
          .in (lo),
          .clz(clz_lo)
      );

      logic hi_all_zero;
      assign hi_all_zero = (hi == {HALF{1'b0}});

      assign clz = hi_all_zero ? (HALF + clz_lo) : clz_hi;

    end
  endgenerate

endmodule



module rx_payload_constructor #(
    parameter int DATA_WIDTH = 512,
    parameter int CONN_ID_WIDTH = 18,
    parameter int IP_UDP_DSCP = 0,
    parameter int IP_UDP_ENC = 0,
    parameter int IP_UDP_IDEN = 0,
    parameter int IP_UDP_FLAGS = 0,
    parameter int IP_UDP_FRAG_OFFSET = 0,
    parameter int IP_UDP_TTL = 64
) (
    input wire rx_axis_aclk,
    input wire rx_axis_aresetn,

    input  wire                     payload_fifo_tlast,
    input  wire                     payload_fifo_tvalid,
    input  wire  [  DATA_WIDTH-1:0] payload_fifo_tdata,
    input  wire  [DATA_WIDTH/8-1:0] payload_fifo_tkeep,
    output logic                    payload_fifo_tready,

    input  wire                            connection_fifo_tlast,
    input  wire                            connection_fifo_tvalid,
    input  wire  [(CONN_ID_WIDTH-1 + 6):0] connection_fifo_tdata,
    output logic                           connection_fifo_tready,

    input  wire        length_check_fifo_out_tlast,
    input  wire        length_check_fifo_out_tvalid,
    input  wire  [7:0] length_check_fifo_out_tdata,
    output logic       length_check_fifo_out_tready,

    input  wire                     payload_out_tready,
    output logic [  DATA_WIDTH-1:0] payload_out_tdata,
    output logic [DATA_WIDTH/8-1:0] payload_out_tkeep,
    output logic                    payload_out_tvalid,
    output logic                    payload_out_tlast
);

  // beginning of a packet behavior:
  // - on the first cycle that all 3 are valid (the 3 fifos)
  // behavior south of the fifo
  typedef enum logic [1:0] {
    IDLE    = 2'b00,
    TRANSFER = 2'b01,
    HOLLOW = 2'b10
  } state_t;
  state_t state;

  // =================================================================
  // Internal FSMs
  // =================================================================
  logic all_fifos_valid;
  logic length_check_pass;
  logic connection_fw_lookup_hit;
  logic [CONN_ID_WIDTH-1:0] connection_fw_lookup_id;

  logic [DATA_WIDTH/8-1:0][7:0] payload_fifo_tdata_bytes;
  logic [DATA_WIDTH/8-1:0][7:0] payload_out_tdata_bytes, payload_out_tdata_bytes_masked;

  // make sure the output is masked with the tkeep
  always_comb begin
    for (int i = 0; i < DATA_WIDTH / 8; i++) begin
      payload_out_tdata_bytes_masked[i] = payload_out_tdata_bytes[i] & {8{payload_out_tkeep[i]}};
    end
  end
  assign payload_out_tdata = payload_out_tdata_bytes_masked;


  logic packet_valid;

  assign connection_fw_lookup_id = connection_fifo_tdata[CONN_ID_WIDTH-1:0];
  assign connection_fw_lookup_hit = connection_fifo_tdata[CONN_ID_WIDTH];
  assign length_check_pass = length_check_fifo_out_tdata[0];

  assign payload_fifo_tdata_bytes = payload_fifo_tdata;
  assign all_fifos_valid = payload_fifo_tvalid && connection_fifo_tvalid && length_check_fifo_out_tvalid;
  assign packet_valid = all_fifos_valid && connection_fw_lookup_hit && length_check_pass;

  always_ff @(posedge rx_axis_aclk) begin
    if (!rx_axis_aresetn) begin
      // reset state
      state <= IDLE;
    end else begin
      case (state)
        IDLE: begin
          if (all_fifos_valid && payload_fifo_tready) begin
            // only go to TRANSFER on valid from length_check and connection_hit 
            if (packet_valid) begin
              if (payload_fifo_tlast) begin
                // No need to transition
                // Output setting is handled in output logic at the end
              end else begin
                state <= TRANSFER;
              end
            end else begin
              if (payload_fifo_tlast) begin
                // No need to transition
                // Output setting is handled in output logic at the end
              end else begin
                state <= HOLLOW;
              end
            end
          end
        end
        TRANSFER: begin
          if (payload_fifo_tvalid && payload_fifo_tready) begin
            if (payload_fifo_tlast) begin
              state <= IDLE;
            end
          end
        end

        HOLLOW: begin
          if (payload_fifo_tvalid && payload_fifo_tready) begin
            if (payload_fifo_tlast) begin
              state <= IDLE;
            end
          end
        end
        default: begin
          // default case
        end
      endcase
    end
  end

  // =================================================================
  // Shift Register FSM
  // =================================================================
  logic [DATA_WIDTH/8-1:0][7:0] payload_reg_data;
  logic [DATA_WIDTH/8-1:0] payload_reg_keep;
  logic payload_reg_last;
  logic payload_reg_valid;
  logic [$clog2(DATA_WIDTH/8):0] payload_reg_rem_count;

  //   assign payload_reg_rem_count = DATA_WIDTH/8 - $countones(payload_reg_keep);


  always_ff @(posedge rx_axis_aclk) begin
    if (!rx_axis_aresetn) begin
      // reset registers
      payload_reg_data <= '0;
      payload_reg_keep <= '0;
      payload_reg_rem_count <= '0;
      payload_reg_last <= 1'b0;
      payload_reg_valid <= 1'b0;
    end else begin
      case (state)
        IDLE: begin
          if (all_fifos_valid) begin
            if (packet_valid) begin
              // start of a new packet
              // if this is the tlast of the packet and reg is busy we save as below, otherwise we still save to reg as below  
              if (payload_reg_valid && !payload_reg_last) begin
                // error condition, we have leftover data in the register that is not last
              end

              // checking for leftover data from a previous packet's last transaction 
              if (payload_reg_valid || (!payload_fifo_tlast)) begin
                // for payload_out bus
                // if (payload_fifio_tlast) begin
                //   // store incoming data into the registers to not drop it
                // // we still output the data from the current registers first
                // end else begin
                //   // if this is not the last transaction, we know that we have to store it in reg still because this will be needed when we transition to transfer
                // end
                payload_reg_valid <= 1'b1;
                payload_reg_last <= payload_fifo_tlast;
                // First 4 bytes store the connection id
                // Second 4 bytes are reserved
                // Rest of the byte carry the MSB 22 bytes from the payload fifo (since this is a first transfer of a packet)
                payload_reg_data[0] <= connection_fw_lookup_id[7:0];
                payload_reg_data[1] <= connection_fw_lookup_id[15:8];
                payload_reg_data[2] <= {6'b0, connection_fw_lookup_id[17:16]};
                payload_reg_data[3] <= 8'b0;
                payload_reg_data[4] <= 8'b0;
                payload_reg_data[5] <= 8'b0;
                payload_reg_data[6] <= 8'b0;
                payload_reg_data[7] <= 8'b0;
                payload_reg_keep[7:0] <= 8'b11111111;
                for (int i = 0; i < FIRST_TRANSACTION_PAYLOAD_SIZE; i++) begin
                  payload_reg_data[i+8] <= payload_fifo_tdata_bytes[i+(HEADER_SIZE_BYTES)];
                  payload_reg_keep[i+8] <= payload_fifo_tkeep[i+(HEADER_SIZE_BYTES)];
                end
                payload_reg_keep[DATA_WIDTH/8-1:8+FIRST_TRANSACTION_PAYLOAD_SIZE] <= {(DATA_WIDTH/8 - (8 + FIRST_TRANSACTION_PAYLOAD_SIZE)) {1'b0}};
                payload_reg_rem_count <= DATA_WIDTH / 8 - (8 + $countones(
                    payload_fifo_tkeep[DATA_WIDTH/8-1:DATA_WIDTH/8-FIRST_TRANSACTION_PAYLOAD_SIZE]
                ));
              end else begin
                // reset registers back to invalid
                payload_reg_data <= '0;
                payload_reg_keep <= '0;
                payload_reg_rem_count <= '0;
                payload_reg_last <= 1'b0;
                payload_reg_valid <= 1'b0;
              end
            end else begin
              // if we don't have an incoming packet,
              // clear the data since it will be output by the next cycle
              payload_reg_data <= '0;
              payload_reg_keep <= '0;
              payload_reg_rem_count <= '0;
              payload_reg_last <= 1'b0;
              payload_reg_valid <= 1'b0;
            end
          end else begin
            // if nothing is coming from the fifos and we're in idle
            // multiple situations:
            // if payload_reg_valid is set but it's not a last, we need to keep it
            // if payload_reg_valid is set and it's last, we can clear it
            if ((payload_reg_valid && payload_reg_last) || (!payload_reg_valid)) begin
              payload_reg_data <= '0;
              payload_reg_keep <= '0;
              payload_reg_rem_count <= '0;
              payload_reg_last <= 1'b0;
              payload_reg_valid <= 1'b0;
            end else begin
            end
          end
        end
        TRANSFER: begin
          if (payload_fifo_tvalid && payload_fifo_tready) begin
            if (payload_reg_valid) begin
              // the register is valid and not yet full
              // we fill in the rest of the register with incoming data depending on how many bytes are in it currently

              // write the remaining data from the incoming data to the reg
              for (int i = 0; i < DATA_WIDTH / 8; i++) begin
                if (i < (DATA_WIDTH / 8 - payload_reg_rem_count)) begin
                  // Load new byte
                  payload_reg_data[i] <= payload_fifo_tdata_bytes[i+payload_reg_rem_count];
                  payload_reg_keep[i] <= payload_fifo_tkeep[i+payload_reg_rem_count];
                end else begin
                  // Keep old value
                  payload_reg_data[i] <= payload_reg_data[i];
                  payload_reg_keep[i] <= payload_reg_keep[i];
                end
              end
              // known widths
              // if the incoming transaction is the last one we don't care about rem count
              // if it's not the last we know that it has to be full then the rem count just fills and refills with the same value
              payload_reg_rem_count <= payload_fifo_tlast ? 0 : payload_reg_rem_count;
              payload_reg_last <= payload_fifo_tlast;
              payload_reg_valid <= 1'b1;
            end else begin
              // if the register is invalid, this is pass-through behavior
              payload_reg_data <= '0;
              payload_reg_keep <= '0;
              payload_reg_rem_count <= 0;
              payload_reg_last <= 1'b0;
              payload_reg_valid <= 1'b0;
            end
          end else begin
            // if not transaction coming in
            if (payload_reg_valid) begin
              if (payload_reg_last) begin
                // if it's last, we can clear it, illegal to append data to that register
                payload_reg_data <= '0;
                payload_reg_keep <= '0;
                payload_reg_rem_count <= 0;
                payload_reg_last <= 1'b0;
                payload_reg_valid <= 1'b0;
              end
            end else begin
              // reg is empty leave it 
            end
          end
        end
        default: begin
          // default case
        end
      endcase
    end
  end


  // =================================================================
  // FIFO Ready Signals
  // =================================================================
  logic can_start_packet;
  assign can_start_packet = (state == IDLE) && all_fifos_valid && payload_out_tready;
  assign payload_fifo_tready = (can_start_packet || (state == TRANSFER) || (state == HOLLOW));
  assign connection_fifo_tready = can_start_packet;
  assign length_check_fifo_out_tready = can_start_packet;


  // =================================================================
  // Output Logic
  // =================================================================
  always_ff @(posedge rx_axis_aclk) begin
    if (!rx_axis_aresetn) begin
      payload_out_tdata_bytes <= '0;
      payload_out_tkeep <= '0;
      payload_out_tvalid <= 1'b0;
      payload_out_tlast <= 1'b0;
    end else begin

      case (state)
        IDLE: begin
          if (all_fifos_valid && packet_valid) begin
            if (payload_reg_valid) begin
              // i have some data in the register
              // if the register is last, output
              if (payload_reg_last) begin
                // output from the register
                payload_out_tdata_bytes <= payload_reg_data;
                payload_out_tkeep <= payload_reg_keep;
                payload_out_tlast <= payload_reg_last;
                payload_out_tvalid <= 1'b1;
              end else begin
                // not last and the register is valid should never happen in IDLE
              end
            end else begin
              // if there's no payload reg data, and this is a first and only transaction for the packet
              // output the connection id + reserved + first payload bytes directly
              // First 4 bytes store the connection id
              // Second 4 bytes are reserved
              // Rest of the byte carry the MSB 22 bytes from the payload fifo (since this is a first transfer of a packet)

              if (payload_fifo_tlast) begin
                payload_out_tvalid <= 1'b1;
                payload_out_tlast <= payload_fifo_tlast;
                // First 4 bytes store the connection id
                // Second 4 bytes are reserved
                // Rest of the byte carry the MSB 22 bytes from the payload fifo (since this is a first transfer of a packet)
                payload_out_tdata_bytes[0] <= connection_fw_lookup_id[7:0];
                payload_out_tdata_bytes[1] <= connection_fw_lookup_id[15:8];
                payload_out_tdata_bytes[2] <= {6'b0, connection_fw_lookup_id[17:16]};
                payload_out_tdata_bytes[3] <= 8'b0;
                payload_out_tdata_bytes[4] <= 8'b0;
                payload_out_tdata_bytes[5] <= 8'b0;
                payload_out_tdata_bytes[6] <= 8'b0;
                payload_out_tdata_bytes[7] <= 8'b0;
                payload_out_tkeep[7:0] <= 8'b11111111;
                for (int i = 0; i < FIRST_TRANSACTION_PAYLOAD_SIZE; i++) begin
                  payload_out_tdata_bytes[i+8] <= payload_fifo_tdata_bytes[i+(HEADER_SIZE_BYTES)];
                  payload_out_tkeep[i+8] <= payload_fifo_tkeep[i+(HEADER_SIZE_BYTES)];
                end
                payload_out_tkeep[DATA_WIDTH/8-1:8+FIRST_TRANSACTION_PAYLOAD_SIZE] <= {(DATA_WIDTH/8 - (8 + FIRST_TRANSACTION_PAYLOAD_SIZE)) {1'b0}};
              end else begin
                // shouldn't reach here since we have a reg to accumulate data until we fill the buffer at least once or hit a tlast,
                // whichver is first
                payload_out_tdata_bytes <= '0;
                payload_out_tkeep <= '0;
                payload_out_tvalid <= 1'b0;
                payload_out_tlast <= 1'b0;
              end
            end
          end else begin
            // fifo empty while we're in idle

            // we only flush from the reg when it's valid with either tlast or full
            if (payload_reg_valid && (payload_reg_last)) begin
              // output from the register
              for (int i = 0; i < DATA_WIDTH / 8; i++) begin
                payload_out_tdata_bytes[i] <= payload_reg_data[i];
                payload_out_tkeep[i] <= payload_reg_keep[i];
              end
              payload_out_tvalid <= 1'b1;
              payload_out_tlast  <= payload_reg_last;
            end else begin
              // there's no valid data to output drive the output low
              payload_out_tdata_bytes <= '0;
              payload_out_tkeep <= '0;
              payload_out_tvalid <= 1'b0;
              payload_out_tlast <= 1'b0;
            end
          end
        end

        TRANSFER: begin
          // transaction
          if (payload_fifo_tvalid && payload_fifo_tready) begin
            if (payload_reg_valid) begin
              // the register is valid and not full
              // we output the bottom bytes from the current transaction + the rest from the register


              // occupied bytes in the register = DATA_WIDTH/8 - payload_reg_rem_count
              for (int i = 0; i < DATA_WIDTH / 8; i++) begin

                if (i < DATA_WIDTH / 8 - payload_reg_rem_count) begin
                  // First X bytes from register
                  payload_out_tdata_bytes[i] <= payload_reg_data[i];
                  payload_out_tkeep[i]       <= payload_reg_keep[i];

                end else begin
                  payload_out_tdata_bytes[i] <= payload_fifo_tdata_bytes[i - (DATA_WIDTH/8 - payload_reg_rem_count)];
                  payload_out_tkeep[i]       <= payload_fifo_tkeep[i - (DATA_WIDTH/8 - payload_reg_rem_count)];
                end
              end

              // if the incoming transaction is too small to require another stage we can have this be tlast = 1
              // if the next one is tlast = 1 and the length fits in the remaining space

              payload_out_tvalid <= 1'b1;
              payload_out_tlast  <= payload_reg_last;
            end else begin
              // no pending data in the register, pass-through behavior
              payload_out_tdata_bytes <= payload_fifo_tdata_bytes;
              payload_out_tkeep <= payload_fifo_tkeep;
              payload_out_tvalid <= 1'b1;
              payload_out_tlast <= payload_fifo_tlast;
            end
          end else begin
            // no transaction
            // but potential full register
            if (payload_reg_valid) begin
              if (payload_reg_last) begin
                // output from the register
                for (int i = 0; i < DATA_WIDTH / 8; i++) begin
                  payload_out_tdata_bytes[i] <= payload_reg_data[i];
                  payload_out_tkeep[i] <= payload_reg_keep[i];
                end
                payload_out_tvalid <= 1'b1;
                payload_out_tlast  <= payload_reg_last;
              end else begin
                // not last and not full do nothing
                // need to wait for more data or a tlast
                payload_out_tdata_bytes <= '0;
                payload_out_tkeep <= '0;
                payload_out_tvalid <= 1'b0;
                payload_out_tlast <= 1'b0;
              end
            end else begin
              // invalid and no transaction do nothing
              payload_out_tdata_bytes <= '0;
              payload_out_tkeep <= '0;
              payload_out_tvalid <= 1'b0;
              payload_out_tlast <= 1'b0;
            end
          end
        end

        HOLLOW: begin
          payload_out_tdata_bytes <= '0;
          payload_out_tkeep <= '0;
          payload_out_tvalid <= 1'b0;
          payload_out_tlast <= 1'b0;
        end
      endcase
    end
  end

endmodule

`default_nettype wire
