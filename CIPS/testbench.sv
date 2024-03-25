// Code your testbench here
// or browse Examples
module testbench;
  
  CIPS cips(clk,R,PC_out, memory_out, adder_out, Output);
  
  reg clk;
  reg R;
  wire[7:0] PC_out, adder_out;
  wire[10:0] memory_out;
  wire[4:0] Output;
  
  initial begin clk = 1'd0; R = 1; end
  initial forever #10 clk=~clk;
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
    
    #15 R = 0;
    
    #150 $finish;
  end
  
endmodule
  
  
  
