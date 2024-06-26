// Code your design here
//D Type Flip-Flop
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

module 
