`default_nettype none
`timescale 1ns/1ps

//
// CMAC Core Bring Up Sequence (With AXI4-Lite Interface)
//
// Assuming TX/RX flow control is disabled by Vivado
// 1. Write the following registers:
// 0x00014 : 32'h00000001 [CONFIGURATION_RX_REG1 for ctl_rx_enable]
// 0x0000C : 32'h00000010 [CONFIGURATION_TX_REG1 for ctl_tx_send_rfi]
// 2. Wait for RX_aligned then write the below registers:
// 0x0000C : 32'h00000001 [CONFIGURATION_TX_REG1 for ctl_tx_enable to 1'b1 and ctl_tx_send_rfi to 1'b0]
//
// For more information refer to:
// UltraScale+ Devices Integrated 100G Ethernet Subsystem 
// v3.1 LogiCORE IP Product Guide Vivado Design Suite 
// PG203 (v3.1) July 17, 2024
//
// AXI4 Interface User Logic:
// The user state machine provides s_axi_awvalid, s_axi_awaddr, s_axi_wvalid, s_axi_wdata and s_axi_wstrb 
// in this state to write to the register map through AXI. When s_axi_bvalid and s_axi_bready from AXI 
// slave are High, it moves to ACK_STATE. If any write operation happens in any illegal addresses, the
// s_axi_bresp[1:0] indicates 2'b10 that asserts the write error signal
//

/*
 *  Author:          M.Subhi Abordan (msubhi_a@mit.edu)
 *  Last Modified:   Nov 11, 2025
 */


module cmac_bring_up_fms (
    input wire rx_aligned,
    input wire trigger_bring_up_seq,

    output logic err_out,
    output logic success_out,

    // AXI Control
    input wire m_axi_aclk,              // AXI clock signal
    input wire m_axi_sreset,            // AXI active-High synchronous reset

    // Address Write Channel
    output logic [31:0] m_axi_awaddr,   // AXI write address
    output logic        m_axi_awvalid,  // AXI write address valid
    input  wire         m_axi_awready,  // AXI write address ready

    // Write Channel
    output logic [31:0] m_axi_wdata,    // AXI write data
    output logic [3:0]  m_axi_wstrb,    // AXI write strobe. This signal indicates which byte lanes hold valid data.
    output logic        m_axi_wvalid,   // AXI write data valid. This signal indicates that valid write data and strobes are available.
    input  wire         m_axi_wready,   // AXI  write data ready

    // Response Channel
    input wire [1:0]    m_axi_bresp,    // AXI write response. This signal indicates the status of the write transaction.
    input wire          m_axi_bvalid,   // AXI write response valid. This signal indicates that the channel is signaling a valid write response.
    output logic        m_axi_bready    // AXI write response ready.
);

    typedef enum {IDLE, WRITE_1, WRITE_2, READ_3, WRITE_4, DONE, ERROR} state_t;
    state_t state;

    always_comb begin
        case (state)
            WRITE_1: begin
                m_axi_awaddr = 32'h00014;
                m_axi_wdata  = 32'h00000001;
            end
            WRITE_2: begin
                m_axi_awaddr = 32'h0000C;
                m_axi_wdata  = 32'h00000010;
            end
            WRITE_4: begin
                m_axi_awaddr = 32'h0000C;
                m_axi_wdata  = 32'h00000001;
            end
            default: begin
                m_axi_awaddr = 32'h0;
                m_axi_wdata  = 32'h0;
            end
        endcase
    end

    // debug signals
    assign err_out      = (state == ERROR);
    assign success_out  = (state == DONE);
    assign m_axi_wstrb  = 4'hF;
    assign m_axi_bready = 1'b1;

    always_ff @(posedge m_axi_aclk) begin

        if (dest_rst) begin
            state           <= IDLE;
            m_axi_awvalid   <= 1'b0;
            m_axi_wvalid    <= 1'b0;
        end else begin

            case (state)
                IDLE: begin
                    state           <= (trigger_bring_up_seq)? WRITE_1: IDLE;
                    m_axi_awvalid   <= trigger_bring_up_seq;
                    m_axi_wvalid    <= trigger_bring_up_seq;
                end

                WRITE_1: begin
                    if (m_axi_bvalid) begin
                        if (m_axi_bresp == 2'b00) begin
                            state           <= WRITE_2;
                            m_axi_awvalid   <= 1'b1;
                            m_axi_wvalid    <= 1'b1;
                        end else begin
                            state           <= ERROR;
                            m_axi_awvalid   <= 1'b0;
                            m_axi_wvalid    <= 1'b0;
                        end
                    end else begin
                        state           <= WRITE_1;
                        m_axi_awvalid   <= 1'b1;
                        m_axi_wvalid    <= 1'b1;
                    end
                end

                WRITE_2: begin
                    if (m_axi_bvalid) begin
                        if (m_axi_bresp == 2'b00) begin
                            state           <= READ_3;
                            m_axi_awvalid   <= 1'b0;
                            m_axi_wvalid    <= 1'b0;
                        end else begin
                            state           <= ERROR;
                            m_axi_awvalid   <= 1'b0;
                            m_axi_wvalid    <= 1'b0;
                        end
                    end else begin
                        state           <= WRITE_2;
                        m_axi_awvalid   <= 1'b1;
                        m_axi_wvalid    <= 1'b1;
                    end
                end

                READ_3: begin
                    if (rx_aligned) begin
                        state           <= WRITE_4;
                        m_axi_awvalid   <= 1'b1;
                        m_axi_wvalid    <= 1'b1;
                    end
                end

                WRITE_4: begin
                    if (m_axi_bvalid) begin
                        if (m_axi_bresp == 2'b00) begin
                            state           <= DONE;
                            m_axi_awvalid   <= 1'b0;
                            m_axi_wvalid    <= 1'b0;
                        end else begin
                            state           <= ERROR;
                            m_axi_awvalid   <= 1'b0;
                            m_axi_wvalid    <= 1'b0;
                        end
                    end else begin
                        state           <= WRITE_4;
                        m_axi_awvalid   <= 1'b1;
                        m_axi_wvalid    <= 1'b1;
                    end
                end

                default: begin
                    m_axi_awvalid   <= 1'b0;
                    m_axi_wvalid    <= 1'b0;
                end
            endcase

        end
    end

endmodule

`default_nettype wire