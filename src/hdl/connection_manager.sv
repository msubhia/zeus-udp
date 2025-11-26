`default_nettype none
`timescale 1ns/1ps


function automatic logic [15:0] xor32to16(input logic [31:0] x);
    return x[31:16] ^ x[15:0];
endfunction


module connection_manager #(
    parameter int KEY_WIDTH   = 32,
    parameter int WAYS        = 4,
    parameter int HASH_WIDTH  = 16,
    parameter int TAG_WIDTH   = KEY_WIDTH,
    parameter int INDEXES     = 1 << HASH_WIDTH,
    parameter int RESP_WIDTH  = HASH_WIDTH+$clog2(WAYS)
)(
    input  wire clk,
    input  wire rst,

    // Forward lookup
    input  wire                      s00_axis_fw_lookup_valid,
    input  wire [KEY_WIDTH-1:0]      s00_axis_fw_lookup_key,
    output logic                     s00_axis_fw_lookup_ready,

    input  wire                      m00_axis_fw_lookup_ready,
    output logic                     m00_axis_fw_lookup_valid,
    output logic                     m00_axis_fw_lookup_hit,
    output logic [RESP_WIDTH-1:0]    m00_axis_fw_lookup_resp,

    // Control port
    input  wire                      s02_axis_ctrl_valid,
    input  wire [KEY_WIDTH-1:0]      s02_axis_ctrl_key,
    input  wire                      s02_axis_ctrl_activate,
    output logic                     s02_axis_ctrl_ready,

    input  wire                      m02_axis_ctrl_ready,
    output logic                     m02_axis_ctrl_valid,
    output logic                     m02_axis_ctrl_ack,
    output logic                     m02_axis_ctrl_full
);

    localparam int WAYS_LOG = $clog2(WAYS);

    // -------------------------------------------------------------------------
    // Hash & Pipeline
    // -------------------------------------------------------------------------
    logic [HASH_WIDTH-1:0] fw_hash;
    assign fw_hash = xor32to16(s00_axis_fw_lookup_key);

    // Pipeline key/hash/valid for BRAM read latency
    logic [1:0][KEY_WIDTH-1:0]      fw_key_pipe;
    logic [1:0][HASH_WIDTH-1:0]     fw_hash_pipe;
    logic [1:0]                     fw_valid_pipe;

    always_ff @(posedge clk) begin
        fw_key_pipe[0]  <= s00_axis_fw_lookup_key;
        fw_key_pipe[1]  <= fw_key_pipe[0];

        fw_hash_pipe[0] <= fw_hash;
        fw_hash_pipe[1] <= fw_hash_pipe[0];

        fw_valid_pipe[0] <= s00_axis_fw_lookup_valid;
        fw_valid_pipe[1] <= fw_valid_pipe[0];
    end

    // -------------------------------------------------------------------------
    // Per-way BRAM arrays
    // -------------------------------------------------------------------------
    logic [TAG_WIDTH-1:0]  fw_tag           [WAYS];
    logic                  fw_valid         [WAYS];

    logic                  ctrl_wren        [WAYS];
    logic [HASH_WIDTH-1:0] ctrl_addr        [WAYS];
    logic [TAG_WIDTH-1:0]  ctrl_din_tag     [WAYS];
    logic                  ctrl_din_valid   [WAYS];
    logic [TAG_WIDTH-1:0]  ctrl_dout_tag    [WAYS];
    logic                  ctrl_dout_valid  [WAYS];

    generate
        for (genvar w=0; w<WAYS; w++) begin : MEMS

            // Tag memory
            dual_port_bram #(
                .DATA_WIDTH(TAG_WIDTH),
                .DATA_DEPTH(INDEXES)
            ) tag_array (
                .clka(clk),
                .rsta(rst),
                .ena(1'b1),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr[w]),
                .dina(ctrl_din_tag[w]),
                .douta(ctrl_dout_tag[w]),

                .clkb(clk),
                .rstb(rst),
                .enb(1'b1),
                .web(1'b0),
                .addrb(fw_hash),
                .dinb('0),
                .doutb(fw_tag[w])
            );

            // Valid bit memory
            dual_port_bram #(
                .DATA_WIDTH(1),
                .DATA_DEPTH(INDEXES)
            ) valid_array (
                .clka(clk),
                .rsta(rst),
                .ena(1'b1),
                .wea(ctrl_wren[w]),
                .addra(ctrl_addr[w]),
                .dina(ctrl_din_valid[w]),
                .douta(ctrl_dout_valid[w]),

                .clkb(clk),
                .rstb(rst),
                .enb(1'b1),
                .web(1'b0),
                .addrb(fw_hash),
                .dinb('0),
                .doutb(fw_valid[w])
            );
        end
    endgenerate



    // -------------------------------------------------------------------------
    // Lookup logic
    // -------------------------------------------------------------------------
    assign s00_axis_fw_lookup_ready = 1'b1;
    assign m00_axis_fw_lookup_valid = fw_valid_pipe[1];

    always_comb begin
        m00_axis_fw_lookup_hit  = 1'b0;
        m00_axis_fw_lookup_resp = '0;

        for (int w = 0; w < WAYS; w++) begin
            if (fw_valid[w] && fw_tag[w] == fw_key_pipe[1]) begin
                m00_axis_fw_lookup_hit = 1'b1;
                m00_axis_fw_lookup_resp = { w[WAYS_LOG-1:0], fw_hash_pipe[1] };
            end
        end
    end



    // -------------------------------------------------------------------------
    // Control FSM
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATE_IDLE,
        STATE_READ,
        STATE_ACTIVATE_CHECK,
        STATE_ACTIVATE,
        STATE_DEACTIVATE
    } state_t;

    state_t state;

    logic [KEY_WIDTH-1:0]       ctrl_key_q;
    logic [HASH_WIDTH-1:0]      ctrl_hash_q;
    logic                       ctrl_act_q;
    logic [WAYS_LOG-1:0]        way_iter;
    logic [2:0]                 read_wait;

    assign s02_axis_ctrl_ready = (state == STATE_IDLE);

    always_ff @(posedge clk) begin
        if (rst) begin
            state               <= STATE_IDLE;
            m02_axis_ctrl_valid <= 1'b0;
            m02_axis_ctrl_ack   <= 1'b0;
            m02_axis_ctrl_full  <= 1'b0;
            way_iter            <= '0;
            read_wait           <= '0;
            for (int i=0; i<WAYS; i++) begin
                ctrl_wren[i]      <= 1'b0;
                ctrl_addr[i]      <= '0;
                ctrl_din_tag[i]   <= '0;
                ctrl_din_valid[i] <= 1'b0;
            end
        end else begin
            // DEFAULTS
            for (int i=0; i<WAYS; i++) begin
                ctrl_wren[i]      <= 1'b0;
                ctrl_addr[i]      <= ctrl_hash_q;
                ctrl_din_tag[i]   <= '0;
                ctrl_din_valid[i] <= 1'b0;
            end

            case (state)

                // -------------------------------------------------------------
                STATE_IDLE: begin
                    m02_axis_ctrl_valid <= 1'b0;
                    m02_axis_ctrl_ack   <= 1'b0;
                    m02_axis_ctrl_full  <= 1'b0;
                    ctrl_key_q          <= s02_axis_ctrl_key;
                    ctrl_hash_q         <= xor32to16(s02_axis_ctrl_key);
                    ctrl_act_q          <= s02_axis_ctrl_activate;
                    read_wait           <= 0;
                    way_iter            <= 0;
                    state               <= (s02_axis_ctrl_valid)? STATE_READ: STATE_IDLE;
                end

                // -------------------------------------------------------------
                STATE_READ: begin
                    read_wait <= read_wait + 1;
                    if (read_wait == 4) begin
                        state <= ctrl_act_q ? STATE_ACTIVATE_CHECK : STATE_DEACTIVATE;
                    end  
                end

                // -------------------------------------------------------------
                STATE_DEACTIVATE: begin
                    for (int i=0; i<WAYS; i++) begin
                        if (ctrl_dout_tag[i] == ctrl_key_q) begin
                            ctrl_wren[i]      <= 1'b1;
                            ctrl_din_valid[i] <= 1'b0;
                            ctrl_din_tag[i]   <= '0;
                        end
                    end
                    m02_axis_ctrl_valid <= 1'b1;
                    m02_axis_ctrl_ack   <= 1'b1;
                    state               <= STATE_IDLE;
                end

                STATE_ACTIVATE_CHECK: begin

                    logic local_is_any;
                    local_is_any = 1'b0;
                    for (int i=0; i<WAYS; i++) begin
                        if (ctrl_dout_tag[i] == ctrl_key_q && ctrl_dout_valid[i]) begin
                            local_is_any = 1'b1;
                        end
                    end

                    if (local_is_any) begin
                        m02_axis_ctrl_valid <= 1'b1;
                        m02_axis_ctrl_ack   <= 1'b1;
                        state               <= STATE_IDLE;
                    end else begin
                        state               <= STATE_ACTIVATE;
                    end

                end

                // -------------------------------------------------------------
                STATE_ACTIVATE: begin
                    if (!ctrl_dout_valid[way_iter]) begin
                        ctrl_wren[way_iter]      <= 1'b1;
                        ctrl_din_valid[way_iter] <= 1'b1;
                        ctrl_din_tag[way_iter]   <= ctrl_key_q;

                        m02_axis_ctrl_valid <= 1'b1;
                        m02_axis_ctrl_ack   <= 1'b1;
                        state               <= STATE_IDLE;

                    end else begin
                        if (way_iter == WAYS-1) begin
                            m02_axis_ctrl_valid <= 1'b1;
                            m02_axis_ctrl_ack   <= 1'b1;
                            m02_axis_ctrl_full  <= 1'b1;
                            state <= STATE_IDLE;
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