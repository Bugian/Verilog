// Code your testbench here
// or browse Examples
module testC; 
  
  counter count(clkt, Rt, Ot);
  
  reg clkt;
  reg Rt;
  wire[3:0] Ot;
  
  initial begin clkt = 0;
    			Rt = 1;
  end
    
  initial forever #10 clkt = ~clkt;
    
 
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(0);
    #30 Rt=0;
    
    #100 $finish;
    
  end
  
endmodule

    
    
