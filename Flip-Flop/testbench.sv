// Code your testbench here
// or browse Examples
`timescale 1ns / 1ps

module d_ff_tb;

    reg clk;
    reg reset;
    reg D;
    wire Q;

    // Instantierea modulului D Flip-Flop
    d_ff uut (
        .clk(clk),
        .reset(reset),
        .D(D),
        .Q(Q)
    );

    // Generarea semnalului de ceas
    always #10 clk = ~clk;

    initial begin
        // Initializarea semnalelor
        clk = 0;
        reset = 0;
        D = 0;

        // Resetarea sistemului
        #5 reset = 1;
        #20 reset = 0;

        // Testarea flip-flop-ului D
        #15 D = 1;
        #20 D = 0;
        #20 D = 1;
        #20 D = 0;

        #30 $finish; // Încheie simularea
    end

    // Monitorizarea semnalelor
    initial begin
      	$dumpfile("dump.vcd"); $dumpvars;
        $monitor("Time = %t, Reset = %b, CLK = %b, D = %b, Q = %b", $time, reset, clk, D, Q);
    end

endmodule
