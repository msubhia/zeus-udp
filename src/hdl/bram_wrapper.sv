`default_nettype none `timescale 1ns / 1ps

module dual_port_bram #(
    parameter int DATA_WIDTH,
    parameter int DATA_DEPTH,
    parameter int BRAM_LATENCY,
    parameter MEMORY_PRIMITIVE = "auto",
    parameter int ADDR_WIDTH = $clog2(DATA_DEPTH)
) (
    // write port a
    input wire clka,
    input wire rsta,
    input wire ena,
    input wire wea,
    input wire [ADDR_WIDTH-1:0] addra,
    input wire [DATA_WIDTH-1:0] dina,
    output logic [DATA_WIDTH-1:0] douta,

    // read port b
    input wire clkb,
    input wire rstb,
    input wire enb,
    input wire web,
    input wire [ADDR_WIDTH-1:0] addrb,
    input wire [DATA_WIDTH-1:0] dinb,
    output logic [DATA_WIDTH-1:0] doutb
);

  // xpm_memory_tdpram: True Dual Port RAM
  // Xilinx Parameterized Macro, version 2025.1

    xpm_memory_tdpram #(
        .ADDR_WIDTH_A(ADDR_WIDTH),               
        .ADDR_WIDTH_B(ADDR_WIDTH),               
        .AUTO_SLEEP_TIME(0),            
        .BYTE_WRITE_WIDTH_A(DATA_WIDTH),        
        .BYTE_WRITE_WIDTH_B(DATA_WIDTH),        
        .CASCADE_HEIGHT(0),             
        .CLOCKING_MODE("independent_clock"), 
        .ECC_BIT_RANGE("7:0"),          
        .ECC_MODE("no_ecc"),            
        .ECC_TYPE("none"),              
        .IGNORE_INIT_SYNTH(0),          
        .MEMORY_INIT_FILE("none"),      
        .MEMORY_INIT_PARAM("0"),        
        .MEMORY_OPTIMIZATION("true"),   
        .MEMORY_PRIMITIVE(MEMORY_PRIMITIVE),      
        .MEMORY_SIZE(DATA_WIDTH * DATA_DEPTH),             
        .MESSAGE_CONTROL(0),            
        .RAM_DECOMP("auto"),            
        .READ_DATA_WIDTH_A(DATA_WIDTH),         
        .READ_DATA_WIDTH_B(DATA_WIDTH),         
        .READ_LATENCY_A(BRAM_LATENCY),             
        .READ_LATENCY_B(BRAM_LATENCY),             
        .READ_RESET_VALUE_A("0"),       
        .READ_RESET_VALUE_B("0"),       
        .RST_MODE_A("SYNC"),            
        .RST_MODE_B("SYNC"),            
        .SIM_ASSERT_CHK(0),
        .USE_EMBEDDED_CONSTRAINT(0),    
        .USE_MEM_INIT(1),               
        .USE_MEM_INIT_MMI(0),           
        .WAKEUP_TIME("disable_sleep"),  
        .WRITE_DATA_WIDTH_A(DATA_WIDTH),        
        .WRITE_DATA_WIDTH_B(DATA_WIDTH),        
        .WRITE_MODE_A("read_first"),     
        .WRITE_MODE_B("read_first"),     
        .WRITE_PROTECT(1)               
    )
    xpm_memory_tdpram_inst (
        .dbiterra(),             // 1-bit output: Status signal to indicate double bit error occurrence on the data output of port A.
        .dbiterrb(),             // 1-bit output: Status signal to indicate double bit error occurrence on the data output of port A.
        .douta(douta),                   // READ_DATA_WIDTH_A-bit output: Data output for port A read operations.
        .doutb(doutb),                   // READ_DATA_WIDTH_B-bit output: Data output for port B read operations.
        .sbiterra(),             // 1-bit output: Status signal to indicate single bit error occurrence on the data output of port A.
        .sbiterrb(),             // 1-bit output: Status signal to indicate single bit error occurrence on the data output of port B.
        .addra(addra),                   // ADDR_WIDTH_A-bit input: Address for port A write and read operations.
        .addrb(addrb),                   // ADDR_WIDTH_B-bit input: Address for port B write and read operations.
        .clka(clka),                     // 1-bit input: Clock signal for port A. Also clocks port B when parameter CLOCKING_MODE is "common_clock".
        .clkb(clkb),                     // 1-bit input: Clock signal for port B when parameter CLOCKING_MODE is "independent_clock". Unused when
                                            // parameter CLOCKING_MODE is "common_clock".

      .dina(dina),  // WRITE_DATA_WIDTH_A-bit input: Data input for port A write operations.
      .dinb(dinb),  // WRITE_DATA_WIDTH_B-bit input: Data input for port B write operations.
      .ena(ena),                       // 1-bit input: Memory enable signal for port A. Must be high on clock cycles when read or write operations
      // are initiated. Pipelined internally.

      .enb(enb),                       // 1-bit input: Memory enable signal for port B. Must be high on clock cycles when read or write operations
      // are initiated. Pipelined internally.

      .injectdbiterra(1'b0), // 1-bit input: Controls double bit error injection on input data when ECC enabled (Error injection capability
      // is not available in "decode_only" mode).

      .injectdbiterrb(1'b0), // 1-bit input: Controls double bit error injection on input data when ECC enabled (Error injection capability
      // is not available in "decode_only" mode).

      .injectsbiterra(1'b0), // 1-bit input: Controls single bit error injection on input data when ECC enabled (Error injection capability
      // is not available in "decode_only" mode).

      .injectsbiterrb(1'b0), // 1-bit input: Controls single bit error injection on input data when ECC enabled (Error injection capability
      // is not available in "decode_only" mode).

      .regcea(1'b1),                 // 1-bit input: Clock Enable for the last register stage on the output data path.
      .regceb(1'b1),                 // 1-bit input: Clock Enable for the last register stage on the output data path.
      .rsta(rsta),                     // 1-bit input: Reset signal for the final port A output register stage. Synchronously resets output port
      // douta to the value specified by parameter READ_RESET_VALUE_A.

      .rstb(rstb),                     // 1-bit input: Reset signal for the final port B output register stage. Synchronously resets output port
      // doutb to the value specified by parameter READ_RESET_VALUE_B.

      .sleep(1'b0),  // 1-bit input: sleep signal to enable the dynamic power saving feature.
      .wea(wea),                       // WRITE_DATA_WIDTH_A/BYTE_WRITE_WIDTH_A-bit input: Write enable vector for port A input data port dina. 1 bit
      // wide when word-wide writes are used. In byte-wide write configurations, each bit controls the writing one
      // byte of dina to address addra. For example, to synchronously write only bits [15-8] of dina when
      // WRITE_DATA_WIDTH_A is 32, wea would be 4'b0010.

      .web(web)                        // WRITE_DATA_WIDTH_B/BYTE_WRITE_WIDTH_B-bit input: Write enable vector for port B input data port dinb. 1 bit
      // wide when word-wide writes are used. In byte-wide write configurations, each bit controls the writing one
      // byte of dinb to address addrb. For example, to synchronously write only bits [15-8] of dinb when
      // WRITE_DATA_WIDTH_B is 32, web would be 4'b0010.
  );  // End of xpm_memory_tdpram_inst instantiation

endmodule  // dual_port_bram

`default_nettype wire
