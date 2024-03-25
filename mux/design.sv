// Code your design here
// Code your design here
module d_ff (
    input clk,     // Semnalul de ceas
    input reset,   // Reset asincron
    input D,       // Date de intrare
    output reg Q   // Ieșirea flip-flop-ului
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            Q <= 1'b0; // Resetarea stării la 0
        else
            Q <= D;    // Actualizarea stării cu D la fiecare front pozitiv al ceasului
    end
endmodule

module mux (
  input [3:0] I,
  input [1:0] S,
  output reg O
);
 always @*
  begin
  case(S)
    2'b00:  O = I[0];
    2'b01:  O = I[1];
    2'b10:  O = I[2];
    2'b11:  O = I[3];
    default: O = 1'bx;
   
  endcase
  
 end
  
endmodule  

module demultiplexor ( input I, input[1:0] S, output reg[3:0] O);
  always @*
  
    begin
      case(S)
      2'b00:  O[0] = I;
      2'b01:  O[1] = I;
      2'b10:  O[2] = I;
      2'b11:  O[3] = I;
      default: O = 4'b0000;
        
      endcase
    
  end
endmodule


    
     
      
  
