// Code your testbench here
// or browse Examples
module testbench;
  
  // Declarațiile semnalelor de ieșire
  wire [31:0] pc_out, adder_out, instr_mem_out, rd1_out, rd2_out, signExtend_out, mux_alu_out, aluData_out, mux_writeData_out, shift_adder_out, aluBranch_out, readData_out, mux_mux_out, muxPC_out;
  wire RegDst_out, jump_out, branch_out, MemRead_out, MemtoReg_out, MemWrite_out, ALUSrc_out, RegWrite_out, gate_out;
  wire [4:0] mux_register_out;
  wire [27:0] shift_mux_out;
  wire [1:0] ALUOp_out;
  wire [2:0] ALU_control_out;
  reg clk, R;
  
  // Instanțierea modulului MIPS
  MIPS dut (.clk(clk), .R(R), .pc_out(pc_out), .adder_out(adder_out), .instr_mem_out(instr_mem_out), 
.rd1_out(rd1_out), .rd2_out(rd2_out), .signExtend_out(signExtend_out), .mux_alu_out(mux_alu_out), 
.aluData_out(aluData_out), .mux_writeData_out(mux_writeData_out), .shift_adder_out(shift_adder_out), 
.aluBranch_out(aluBranch_out), .readData_out(readData_out), .mux_mux_out(mux_mux_out), .muxPC_out(muxPC_out), 
.RegDst_out(RegDst_out), .jump_out(jump_out), .branch_out(branch_out), .MemRead_out(MemRead_out), 
.MemtoReg_out(MemtoReg_out), .MemWrite_out(MemWrite_out), .ALUSrc_out(ALUSrc_out), .RegWrite_out(RegWrite_out), 
.gate_out(gate_out), .mux_register_out(mux_register_out), .shift_mux_out(shift_mux_out), .ALUOp_out(ALUOp_out), 
.ALU_control_out(ALU_control_out), .rd1_out(rd1_out), .rd2_out(rd2_out), .signExtend_out(signExtend_out), 
.aluData_out(aluData_out), .aluBranch_out(aluBranch_out), .zero_out(zero_out));
  
  initial begin clk = 1'd0; R = 1; end
  initial forever #10 clk=~clk;
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
    
    #10 R = 0; // Dezactivează resetul după 10 unități de timp
    
    #1000 $finish; // Termină simularea după 1000 de unități de timp
  end
  
endmodule
