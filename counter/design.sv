// Code your design here
module counter(input clk, input R, output reg[3:0] O);
  always @(posedge clk or posedge reset) begin
    
    if (R) begin
    	O <= 0;
    end
    else begin
      O <= O+1;
    end    
    
  end
  
endmodule
