`default_nettype none
`timescale 1ns/1ps


function automatic logic [15:0] xor32to16(input logic [31:0] x);
    return x[31:16] ^ x[15:0];
endfunction


module connection_manager #(
    parameter int KEY_WIDTH     = 32,
    parameter int WAYS          = 4,
    parameter int HASH_WIDTH    = 16,
    parameter int TAG_WIDTH     = KEY_WIDTH,
    parameter int INDEXES       = 1 << HASH_WIDTH,
    parameter int RESP_WIDTH    = HASH_WIDTH + WAYS_LOG
) (
    input wire clk,
    input wire rst,

    // Forward lookup (key -> resp)
    input wire                      s00_axis_fw_lookup_valid,
    input wire [KEY_WIDTH-1:0]      s00_axis_fw_lookup_key,
    output logic                    s00_axis_fw_lookup_ready,

    input wire                      m00_axis_fw_lookup_ready, // assumes always 1
    output logic                    m00_axis_fw_lookup_valid,
    output logic                    m00_axis_fw_lookup_hit,
    output logic [RESP_WIDTH-1:0]   m00_axis_fw_lookup_resp,
    

    // TODO: SUPPORT Reverse lookup (resp -> key)
    // input  wire [RESP_WIDTH-1:0]    rv_lookup_resp,
    // output logic                    rv_lookup_hit,
    // output logic [KEY_WIDTH-1:0]    rv_lookup_key,


    // Control port: used to write
    input wire                      s02_axis_ctrl_valid,
    input wire [KEY_WIDTH-1:0]      s02_axis_ctrl_key,
    input wire                      s02_axis_ctrl_activate,
    output logic                    s02_axis_ctrl_ready,

    input wire                      m02_axis_ctrl_ready,    // assumes always 1
    output logic                    m02_axis_ctrl_valid,
    output logic                    m02_axis_ctrl_ack,
    output logic                    m02_axis_ctrl_full,
);

    // Hash index
    logic [HASH_WIDTH-1:0] s00_axis_fw_lookup_hashed_key;
    assign s00_axis_fw_lookup_hashed_key = xor32to16(s00_axis_fw_lookup_key);

    // -------------------------------------------------------------------------
    // Storage (BRAM): WAYS x INDEXES x TAG_WIDTH
    // -------------------------------------------------------------------------

    logic [1:0][KEY_WIDTH-1:0] s00_axis_fw_lookup_key_pipe;
    always_ff @(posedge clk) begin
        s00_axis_fw_lookup_key_pipe[0] <= s00_axis_fw_lookup_key;
        s00_axis_fw_lookup_key_pipe[1] <= s00_axis_fw_lookup_key_pipe[0];
    end

    logic [1:0][HASH_WIDTH-1:0] s00_axis_fw_lookup_hashed_key_pipe;
    always_ff @(posedge clk) begin
        s00_axis_fw_lookup_hashed_key_pipe[0] <= s00_axis_fw_lookup_hashed_key;
        s00_axis_fw_lookup_hashed_key_pipe[1] <= s00_axis_fw_lookup_hashed_key_pipe[0];
    end

    logic [1:0] s00_axis_fw_lookup_valid_pipe;
    always_ff @(posedge clk) begin
        s00_axis_fw_lookup_valid_pipe[0] <= s00_axis_fw_lookup_valid;
        s00_axis_fw_lookup_valid_pipe[1] <= s00_axis_fw_lookup_valid_pipe[0];
    end

    generate
        for (genvar w=0; w<WAYS; w=w+1) begin: MEM_ARRAYS

            logic [TAG_WIDTH-1:0] __fw_tag;
            logic __fw_valid;

            dual_port_bram #(
                .DATA_WIDTH(TAG_WIDTH),
                .DATA_DEPTH(INDEXES)
            ) tag_array (
                .clka(clk),
                .rsta(rst),
                .ena(1'b1),
                .wea(),
                .addra(),
                .dina(),
                .douta()

                .clkb(clk),
                .rstb(rst),
                .enb(1'b1),
                .web(1'b0),
                .addrb(s00_axis_fw_lookup_key),
                .dinb('b0),
                .doutb(__fw_tag)
            )

            dual_port_bram #(
                .DATA_WIDTH(1),
                .DATA_DEPTH(INDEXES)
            ) valid_array (
                .clka(clk),
                .rsta(rst),
                .ena(1'b1),
                .wea(),
                .addra(),
                .dina(),
                .douta()

                .clkb(clk),
                .rstb(rst),
                .enb(1'b1),
                .web(1'b0),
                .addrb(s00_axis_fw_lookup_key),
                .dinb('b0),
                .doutb(__fw_valid)
            )

        end
    endgenerate

    // -------------------------------------------------------------------------
    // Lookup
    // -------------------------------------------------------------------------

    assign s00_axis_fw_lookup_ready = 1'b1;
    assign m00_axis_fw_lookup_valid = s00_axis_fw_lookup_valid_pipe[1];

    always_comb begin
        m00_axis_fw_lookup_hit  = 1'b0;
        m00_axis_fw_lookup_resp = '0;
        for (int w = 0; w < WAYS; w++) begin
            if (MEM_ARRAYS.w.__fw_valid && MEM_ARRAYS.w.__fw_tag == fw_hashed_key_pipe[1]) begin
                m00_axis_fw_lookup_hit  |= 1'b1;
                m00_axis_fw_lookup_resp |= { w[WAYS_LOG-1:0], hashed_key };
            end
        end
    end


    // -------------------------------------------------------------------------
    // Control FSM
    // -------------------------------------------------------------------------

    // input wire                      s02_axis_ctrl_valid,
    // input wire [KEY_WIDTH-1:0]      s02_axis_ctrl_key,
    // input wire                      s02_axis_ctrl_activate,
    // output logic                    s02_axis_ctrl_ready,

    // input wire                      m02_axis_ctrl_ready,    // assumes always 1
    // output logic                    m02_axis_ctrl_valid,
    // output logic                    m02_axis_ctrl_ack,
    // output logic                    m02_axis_ctrl_full,

    typedef enum logic [1:0] { IDLE, ACTIVATE, DEACTIVATE } state_t;
    state_t state;


    logic [KEY_WIDTH-1:0]      ctrl_key_q;
    logic [HASH_WIDTH-1:0]     ctrl_hash_q;

    logic [$clog2(WAYS)-1:0]    way_iter;


    assign s02_axis_ctrl_ready = (state != IDLE);


    always_ff @(posedge clk) begin
    end





    always_ff @(posedge clk) begin
        if (rst) begin
            state        <= IDLE;
            ctrl_ack     <= 1'b0;
            ctrl_full    <= 1'b0;
            way_iter     <= '0;
        end else begin
            ctrl_ack <= 1'b0;
            ctrl_full <= 1'b0;

            case (state)

                IDLE: begin
                    if (ctrl_valid) begin
                        ctrl_key_q  <= ctrl_key;
                        ctrl_hash_q <= xor32to16(ctrl_key);
                        way_iter    <= '0;
                        state       <= ctrl_activate ? ACTIVATE : DEACTIVATE;
                    end
                end

                DEACTIVATE: begin
                    for (int w=0; w<WAYS; w++) begin
                        if (tags[w][ctrl_hash_q] == ctrl_key_q) begin
                            valid[w][ctrl_hash_q] <= 1'b0;
                        end 
                    end
                    ctrl_ack <= 1'b1;
                    state <= IDLE;
                end

                ACTIVATE: begin
                    if (!valid[way_iter][ctrl_hash_q]) begin
                        valid[way_iter][ctrl_hash_q] <= 1'b1;
                        tags [way_iter][ctrl_hash_q] <= ctrl_key_q;
                        ctrl_ack <= 1'b1;
                        state <= IDLE;
                    end else begin
                        if (way_iter == WAYS-1) begin
                            ctrl_ack  <= 1'b1;
                            ctrl_full <= 1'b1;
                            state     <= IDLE;
                        end else begin
                            way_iter <= way_iter + 1;
                        end
                    end
                end

            endcase
        end
    end


    




endmodule











module hash_table #(
    parameter int WAYS       = 4,
    parameter int KEY_WIDTH  = 32,
    parameter int HASH_WIDTH = 16,

    parameter int WAYS_LOG   = $clog2(WAYS),
    parameter int TAG_WIDTH  = KEY_WIDTH,

    parameter int INDEXES    = 1 << HASH_WIDTH,
    parameter int RESP_WIDTH = HASH_WIDTH + WAYS_LOG
) (
    input  wire clk,
    input  wire rst,

    // Forward lookup (key -> resp)
    input  wire [KEY_WIDTH-1:0]     fw_lookup_key,
    output logic                    fw_lookup_hit,
    output logic [RESP_WIDTH-1:0]   fw_lookup_resp,

    // Reverse lookup (resp -> key)
    input  wire [RESP_WIDTH-1:0]    rv_lookup_resp,
    output logic                    rv_lookup_hit,
    output logic [KEY_WIDTH-1:0]    rv_lookup_key,

    // Control port
    input  wire                     ctrl_valid,
    input  wire [KEY_WIDTH-1:0]     ctrl_key,
    input  wire                     ctrl_activate,
    output logic                    ctrl_ack,
    output logic                    ctrl_full,
    output logic                    ctrl_busy
);

    // Hash index
    logic [HASH_WIDTH-1:0] hashed_key;
    assign hashed_key = xor32to16(fw_lookup_key);

    // -------------------------------------------------------------------------
    // Storage (BRAM): WAYS x INDEXES x TAG_WIDTH
    // -------------------------------------------------------------------------

    logic [TAG_WIDTH-1:0] tags   [WAYS][INDEXES];
    logic                 valid  [WAYS][INDEXES];

    // -------------------------------------------------------------------------
    // Lookup (combinational)
    // -------------------------------------------------------------------------

    always_comb begin
        fw_lookup_hit  = 1'b0;
        fw_lookup_resp = '0;

        for (int w = 0; w < WAYS; w++) begin
            if (valid[w][hashed_key] && tags[w][hashed_key] == fw_lookup_key) begin
                fw_lookup_hit  |= 1'b1;
                fw_lookup_resp |= { w[WAYS_LOG-1:0], hashed_key };
            end
        end

    end

    // Reverse mapping
    logic [WAYS_LOG-1:0] rv_way;
    logic [HASH_WIDTH-1:0] rv_index;

    assign rv_way   = rv_lookup_resp[RESP_WIDTH-1 : HASH_WIDTH];
    assign rv_index = rv_lookup_resp[HASH_WIDTH-1 : 0];

    always_comb begin
        rv_lookup_hit = valid[rv_way][rv_index];
        rv_lookup_key = tags [rv_way][rv_index];
    end

    // -------------------------------------------------------------------------
    // Control FSM
    // -------------------------------------------------------------------------

    typedef enum logic [1:0] { IDLE, ACTIVATE, DEACTIVATE } state_t;
    state_t state;

    logic [KEY_WIDTH-1:0]      ctrl_key_q;
    logic [HASH_WIDTH-1:0]     ctrl_hash_q;

    logic [WAYS_LOG-1:0] way_iter;

    assign ctrl_busy = (state != IDLE);

    always_ff @(posedge clk) begin
        if (rst) begin
            state        <= IDLE;
            ctrl_ack     <= 1'b0;
            ctrl_full    <= 1'b0;
            way_iter     <= '0;
        end else begin
            ctrl_ack <= 1'b0;
            ctrl_full <= 1'b0;

            case (state)

                IDLE: begin
                    if (ctrl_valid) begin
                        ctrl_key_q  <= ctrl_key;
                        ctrl_hash_q <= xor32to16(ctrl_key);
                        way_iter    <= '0;
                        state       <= ctrl_activate ? ACTIVATE : DEACTIVATE;
                    end
                end

                DEACTIVATE: begin
                    for (int w=0; w<WAYS; w++) begin
                        if (tags[w][ctrl_hash_q] == ctrl_key_q) begin
                            valid[w][ctrl_hash_q] <= 1'b0;
                        end 
                    end
                    ctrl_ack <= 1'b1;
                    state <= IDLE;
                end

                ACTIVATE: begin
                    if (!valid[way_iter][ctrl_hash_q]) begin
                        valid[way_iter][ctrl_hash_q] <= 1'b1;
                        tags [way_iter][ctrl_hash_q] <= ctrl_key_q;
                        ctrl_ack <= 1'b1;
                        state <= IDLE;
                    end else begin
                        if (way_iter == WAYS-1) begin
                            ctrl_ack  <= 1'b1;
                            ctrl_full <= 1'b1;
                            state     <= IDLE;
                        end else begin
                            way_iter <= way_iter + 1;
                        end
                    end
                end

            endcase
        end
    end

endmodule

`default_nettype wire


























// `default_nettype none

// function automatic logic [15:0] xor64to16(input logic [63:0] x);
//     return x[63:48] ^ x[47:32] ^ x[31:16] ^ x[15:0];
// endfunction

// function automatic logic [7:0] xor32to8(input logic [31:0] x);
//     return x[31:24] ^ x[23:16] ^ x[15:8] ^ x[7:0];
// endfunction

// function automatic logic [15:0] xor32to16(input logic [31:0] x);
//     return x[31:16] ^ x[15:0];
// endfunction


// module hash_table #(
//     parameter int WAYS       = 4,
//     parameter int WAYS_LOG   = $clog2(WAYS),
//     parameter int KEY_WIDTH  = 32,
//     parameter int TAG_WIDTH  = KEY_WIDTH,
//     parameter int HASH_WIDTH = 16,
//     parameter int INDEXES    = 1 << HASH_WIDTH,
//     parameter int RESP_WIDTH = HASH_WIDTH + WAYS_LOG
// ) (
//     input  wire clk,
//     input  wire rst,

//     // Forward lookup (key -> resp)
//     input  wire [KEY_WIDTH-1:0]     fw_lookup_key,
//     output logic                    fw_lookup_hit,
//     output logic [RESP_WIDTH-1:0]   fw_lookup_resp,

//     // Reverse lookup (resp -> key)
//     input  wire [RESP_WIDTH-1:0]    rv_lookup_resp,
//     output logic                    rv_lookup_hit,
//     output logic [KEY_WIDTH-1:0]    rv_lookup_key,

//     // Control port
//     input  wire                     ctrl_valid,
//     input  wire [KEY_WIDTH-1:0]     ctrl_key,
//     input  wire                     ctrl_activate,
//     output logic                    ctrl_ack,
//     output logic                    ctrl_full,
//     output logic                    ctrl_busy
// );

//     // Hash index
//     logic [HASH_WIDTH-1:0] hashed_key;
//     assign hashed_key = xor32to16(fw_lookup_key);

//     // -------------------------------------------------------------------------
//     // Storage (BRAM): WAYS x INDEXES x TAG_WIDTH
//     // -------------------------------------------------------------------------

//     logic [TAG_WIDTH-1:0] tags   [WAYS][INDEXES];
//     logic                 valid  [WAYS][INDEXES];

//     // -------------------------------------------------------------------------
//     // Lookup (combinational)
//     // -------------------------------------------------------------------------

//     always_comb begin
//         fw_lookup_hit  = 1'b0;
//         fw_lookup_resp = '0;

//         for (int w = 0; w < WAYS; w++) begin
//             if (valid[w][hashed_key] && tags[w][hashed_key] == fw_lookup_key) begin
//                 fw_lookup_hit  |= 1'b1;
//                 fw_lookup_resp |= { w[WAYS_LOG-1:0], hashed_key };
//             end
//         end

//     end

//     // Reverse mapping
//     logic [WAYS_LOG-1:0] rv_way;
//     logic [HASH_WIDTH-1:0] rv_index;

//     assign rv_way   = rv_lookup_resp[RESP_WIDTH-1 : HASH_WIDTH];
//     assign rv_index = rv_lookup_resp[HASH_WIDTH-1 : 0];

//     always_comb begin
//         rv_lookup_hit = valid[rv_way][rv_index];
//         rv_lookup_key = tags [rv_way][rv_index];
//     end

//     // -------------------------------------------------------------------------
//     // Control FSM
//     // -------------------------------------------------------------------------

//     typedef enum logic [1:0] { IDLE, ACTIVATE, DEACTIVATE } state_t;
//     state_t state;

//     logic [KEY_WIDTH-1:0]      ctrl_key_q;
//     logic [HASH_WIDTH-1:0]     ctrl_hash_q;

//     logic [WAYS_LOG-1:0] way_iter;

//     assign ctrl_busy = (state != IDLE);

//     always_ff @(posedge clk) begin
//         if (rst) begin
//             state        <= IDLE;
//             ctrl_ack     <= 1'b0;
//             ctrl_full    <= 1'b0;
//             way_iter     <= '0;
//         end else begin
//             ctrl_ack <= 1'b0;
//             ctrl_full <= 1'b0;

//             case (state)

//                 IDLE: begin
//                     if (ctrl_valid) begin
//                         ctrl_key_q  <= ctrl_key;
//                         ctrl_hash_q <= xor32to16(ctrl_key);
//                         way_iter    <= '0;
//                         state       <= ctrl_activate ? ACTIVATE : DEACTIVATE;
//                     end
//                 end

//                 DEACTIVATE: begin
//                     for (int w=0; w<WAYS; w++) begin
//                         if (tags[w][ctrl_hash_q] == ctrl_key_q) begin
//                             valid[w][ctrl_hash_q] <= 1'b0;
//                         end 
//                     end
//                     ctrl_ack <= 1'b1;
//                     state <= IDLE;
//                 end

//                 ACTIVATE: begin
//                     if (!valid[way_iter][ctrl_hash_q]) begin
//                         valid[way_iter][ctrl_hash_q] <= 1'b1;
//                         tags [way_iter][ctrl_hash_q] <= ctrl_key_q;
//                         ctrl_ack <= 1'b1;
//                         state <= IDLE;
//                     end else begin
//                         if (way_iter == WAYS-1) begin
//                             ctrl_ack  <= 1'b1;
//                             ctrl_full <= 1'b1;
//                             state     <= IDLE;
//                         end else begin
//                             way_iter <= way_iter + 1;
//                         end
//                     end
//                 end

//             endcase
//         end
//     end

// endmodule

// `default_nettype wire