module testbench;
  
  // Declarations of output signals
  wire [31:0] pc_out, adder_out, instr_mem_out, rd1_out, rd2_out, signExtend_out, mux_alu_out, aluData_out, mux_writeData_out, shift_adder_out, aluBranch_out, readData_out, mux_mux_out, muxPC_out;
  wire RegDst_out, jump_out, branch_out, MemRead_out, MemtoReg_out, MemWrite_out, ALUSrc_out, RegWrite_out, gate_out, zero_out;
  wire [4:0] mux_register_out;
  wire [27:0] shift_mux_out;
  wire [1:0] ALUOp_out;
  wire [2:0] ALU_control_out;
  reg clk, R;
  
  // Instantiation of the MIPS module
  MIPS mipsProcessor(
    .clk(clk), 
    .R(R), 
    .pc_out(pc_out), 
    .adder_out(adder_out), 
    .instr_mem_out(instr_mem_out), 
    .rd1_out(rd1_out), 
    .rd2_out(rd2_out), 
    .signExtend_out(signExtend_out), 
    .mux_alu_out(mux_alu_out), 
    .aluData_out(aluData_out), 
    .mux_writeData_out(mux_writeData_out), 
    .shift_adder_out(shift_adder_out), 
    .aluBranch_out(aluBranch_out), 
    .readData_out(readData_out), 
    .mux_mux_out(mux_mux_out), 
    .muxPC_out(muxPC_out), 
    .RegDst_out(RegDst_out), 
    .jump_out(jump_out), 
    .branch_out(branch_out), 
    .MemRead_out(MemRead_out), 
    .MemtoReg_out(MemtoReg_out), 
    .MemWrite_out(MemWrite_out), 
    .ALUSrc_out(ALUSrc_out), 
    .RegWrite_out(RegWrite_out), 
    .gate_out(gate_out), 
    .mux_register_out(mux_register_out), 
    .shift_mux_out(shift_mux_out), 
    .ALUOp_out(ALUOp_out), 
    .ALU_control_out(ALU_control_out)
  );
  
  initial begin 
    clk = 1'b0; 
    R = 1'b1; // Reset is active-high
  end
  
  // Clock generation
  always #5 clk = ~clk;
  
  // Activate reset for a brief period and then deactivate it
  initial begin
    #10;
    R = 1'b0; // Deactivate reset
    #1000;
    R = 1'b1; // Activate reset
    #10;
    R = 1'b0; // Deactivate reset
  end
  
  // Dump waveform to a VCD file
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(0, testbench); // Dump all variables in the testbench module
  end
  
  // Stop simulation after certain time
  initial #2000 $finish;
  
endmodule

