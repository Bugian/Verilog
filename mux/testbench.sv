// Code your testbench here
// or browse Examples
module testbench;
  
  mux Mux( It, St, Ot);
  
  reg[3:0] It;
  reg[1:0] St;
  wire Ot;
  
  demultiplexor demux (Idt, Sdt, Odt);
  
  reg Idt;
  reg[1:0] Sdt;
  wire[3:0] Odt;
  
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(1);
    
    #5 It = 4'b0110;
   
    #5 St = 2'b00;
    #5 St = 2'b01;
    #5 St = 2'b10;
    #5 St = 2'b11;
    
    #10 $finish;
    
  
  end
  
  initial begin
   
    
    #5 Idt = 1'b1;
   
    #5 Sdt = 2'b00;
    #5 Sdt = 2'b01;
    #5 Sdt = 2'b10;
    #5 Sdt = 2'b11;
    
    #10 $finish;
    
  
  end
  
endmodule
  
