
`timescale 1ns / 1ps
module cocotb_vivado_dump();
  initial begin
    $dumpfile("/home/msubhi_a/Documents/zeus-rpc/src/sim/sim_build/connection_manager_wrapper.fst");
    $dumpvars(0,connection_manager_wrapper);
  end
endmodule
