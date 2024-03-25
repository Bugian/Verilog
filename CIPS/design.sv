// Code your design here
module ALU(input[3:0] x, input[3:0] y, input[2:0] control, output reg[4:0] Out);
  
  always @*
    
  casex(control)
    
    3'b000:
    	Out <= x + y;
    3'b001:
    	Out <= x - y;
    3'b010:
    	Out <= x | y;
    3'b011:
    	Out <= x & y;
    3'b100:
    	Out <= x ^ y;
    default: 
      Out = 0;
	endcase
    
endmodule

module instructionMemory (input[7:0] In, output reg[10:0] Out);
  
  reg[10:0] memory[0:1023];
  initial begin
    $readmemb("instr1.txt", memory);
  end
  
  always @*
    begin
      
      Out = memory[In];
      
    end
  
endmodule

module adder(input[7:0] In, output reg[7:0] Out);

  always @*
  begin
    Out <= In + 1;
  end
  
endmodule

module PC(input[7:0] In, input clk, input R, output reg[7:0] Out);
  
  always @(posedge clk or posedge R) begin
    
    if (R) 
      Out <= 8'd0;
    else 
      Out <= In;
 
      
 end
endmodule


module CIPS(input clk, input R, inout[7:0] PC_out, inout[10:0] memory_out , inout[7:0] adder_out, output wire[4:0] Output);
  
  PC PCount(.In(adder_out), .clk(clk), .R(R), .Out(PC_out));
  adder add(.In(PC_out), .Out(adder_out));
  instructionMemory instrMem(.In(PC_out), .Out(memory_out));
  ALU alu(.x(memory_out[7:4]), .y(memory_out[3:0]), .control(memory_out[10:8]), 	.Out(Output));
  
endmodule
  
