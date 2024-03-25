// Code your design here
module registers(rs, rt, write_r, write_d, read_data1, read_data2, RegWrite, clk);
  input[4:0] rs, rt, write_r;
  input[31:0] write_d;
  input RegWrite, clk;
  output reg[31:0] read_data1, read_data2;
  
  reg[31:0] registri[0:31];
  integer i;
  
  initial begin
    for(i = 0; i < 32; i = i + 1) begin
      registri[i] <= 32'd0;
    end
  end
  
  always @(rt, rs,posedge clk) begin
  	read_data1 = rs;
    read_data2 = rt;
  end
  
  always @(negedge clk) begin
    if (RegWrite)
      registri[write_r] <= write_d;
  end
  
  
endmodule


module instruction_mem(read_address, instruction);
  input[31:0] read_address;
  output reg[31:0] instruction;
  
  reg[31:0] memory[0:1023];
  initial begin
    $readmemb("instr.txt", memory);
  end
  
  always @* begin
    instruction = memory[read_address>>2];
  end
endmodule

module Control(control_in, regDst, Jump, Branch, MemRead, MemToReg, ALUOp, MemWrite, ALUSrc, RegWrite);
  
  input[5:0] control_in;
  output reg regDst, Jump, Branch, MemRead, MemToReg, ALUSrc, MemWrite, RegWrite;
  output reg[1:0] ALUOp;
  
 always @* begin
  
  case(control_in) 
	
    6'd0: begin //R type
    regDst <= 1'b1; Jump <= 1'b0; Branch <= 1'b0; MemRead <= 1'b0; MemToReg <= 1'b0; 		ALUOp <= 2'b00; MemWrite <= 1'b0; ALUSrc <= 1'b0; RegWrite <= 1'b1;
    end
    
    6'b01000: begin //addi
    regDst <= 1'b0; Jump <= 1'b0; Branch <= 1'b0; MemRead <= 1'b0; MemToReg <= 1'b0; 		ALUOp <= 2'b01; MemWrite <= 0; ALUSrc <= 1; RegWrite <= 1;
    end
    
    6'b000100: begin //brench on equal (beq)
    regDst <= 1'bx; Jump <= 1'b0; Branch <= 1'b1; MemRead <= 1'b0; MemToReg <= 1'bx;         ALUOp <= 2'b10; MemWrite <= 1'b0; ALUSrc <= 1'b0; RegWrite <= 1'b0;  
    end
    
    6'b000010: begin //Jump
    regDst <= 1'bx; Jump <= 1'b1; Branch <= 1'bx; MemRead <= 1'b0; MemToReg <= 1'bx;         ALUOp <= 2'bxx; MemWrite <= 1'b0; ALUSrc <= 1'bx; RegWrite <= 1'b0;
    end
    
    6'b100011: begin //lw
    regDst <= 1'b0; Jump <= 1'b0; Branch <= 1'b0; MemRead <= 1'b1; MemToReg <= 1'b1;         ALUOp <= 2'b01; MemWrite <= 1'b0; ALUSrc <= 1'b1; RegWrite <= 1'b1;
    end
    
    6'b101011: begin
    regDst <= 1'bx; Jump <= 1'b0; Branch <= 1'b0; MemRead <= 1'b1; MemToReg <= 1'bx;         ALUOp <= 2'b01; MemWrite <= 1'b1; ALUSrc <= 1'b1; RegWrite <= 1'b0;
    end
    endcase
  end
endmodule

module program_counter(in, out, R, clk);
  input[31:0] in;
  input clk, R;
  output reg[31:0] out;
  
  always @(posedge clk or R) begin
    if(R)
      out <= 32'd0;
    else
      out <= in;
  end
endmodule

module shift_left(in, out);
  input[31:0] in;
  output reg[31:0] out;
  
  always @*
    out = in << 2;
endmodule

module shift_left_26_28(ins, outs);
  input[25:0] ins;
  output reg[27:0] outs;
  
  always @* begin
    outs <= {ins, 2'b00};
  end
endmodule
  
module ALUControl(instruction, ALUOp, ALU_out);
  input[4:0] instruction;
  input[1:0] ALUOp;
  output reg[2:0] ALU_out;
  
  always @* begin
    case(ALUOp) 
      2'b00:
        case(instruction) 
          6'b100000: ALU_out <= 3'b000; //add
          6'b100010: ALU_out <= 3'b001; //subtract
          6'b100100: ALU_out <= 3'b010; //and
          6'b100101: ALU_out <= 3'b011; //or
          6'b100111: ALU_out <= 3'b100; //nor
          default: ALU_out <= 3'bxxx;
        endcase
      2'b01:ALU_out <= 3'b000;
      2'b10:ALU_out <= 3'b001;
    endcase 
  end
endmodule

module ALU(x, y, alu_control, alu_out, zero_out);
  
  input[31:0] x, y;
  input[2:0] alu_control;
  output reg zero_out;
  output reg[31:0] alu_out;
  
  always @* begin
    case(alu_control)
      3'b000: zero_out = 1; 
      3'b001: alu_out = x + y;
      3'b010: alu_out = x - y;
      3'b011: alu_out = x & y;
      3'b100: alu_out = x | y;
      3'b101: alu_out = ~(x | y);
      endcase
  end
endmodule

module DataMemory(alu_result, rd2, mem_read, mem_write, read_data, clk);
  input[31:0] alu_result, rd2;
  input mem_read, mem_write, clk;
  output reg[31:0] read_data;
  
  reg[31:0] memory[127:0];
  
  always @(negedge clk) begin
    
  if(mem_read)
    read_data <= memory[alu_result[31:2]];
  
  if (mem_write)
    memory[alu_result[31:2]] <= rd2;
  end
endmodule
               
module mux_32(x, y, S, out);

  input[31:0] x, y;
  input S;
  output reg[31:0] out;
  
  always @* begin
    
    if(S)
      out = x;
    else
      out = y;
  end
  
  
endmodule
               
module mux_5(x, y, S, out);

  input[4:0] x, y;
  input S;
  output reg[4:0] out;
  
  always @* begin
    
    if(S)
      out = x;
    else
      out = y;
  end
  
  
endmodule
  
module shift_left2(in, out);
  input[31:0] in;
  output reg[31:0] out;
  
  always @* begin
  
    out = in<<2;
    
  end
  
endmodule
               
module shift_left26_28(in, out);
  input[25:0] in;
  output reg[27:0] out;
  
  always @* begin
  
    out = in<<2;
    
  end
  
endmodule
               
module signExtend(in, out);
  
  input[15:0] in;
  output reg[31:0] out;
  
  always @* begin
    out <= in;
  end
  
endmodule
               
module adderCounter(pc_out, out);
  
  input[31:0] pc_out;
  output reg[31:0] out;
  
  always @* begin
    out = pc_out + 4;
    
  end
  
endmodule
               
module adder(in1, in2, out);
  
  input[31:0] in1, in2;
  output reg[31:0] out;
  
  always @* begin
    out = in1 + in2;
    
  end
  
endmodule
                 
                 
module MIPS(clk, R, pc_out, adder_out, instr_mem_out, RegDst_out, jump_out, branch_out, MemRead_out, MemtoReg_out, ALUOp_out, MemWrite_out, ALUSrc_out, RegWrite_out, mux_register_out, mux_alu_out, mux_writeData_out, mux_mux_out, muxPC_out, shift_mux_out, shift_adder_out, ALU_control_out, rd1_out, rd2_out, signExtend_out, aluData_out, aluBranch_out, zero_out, gate_out, readData_out);
  
  inout[31:0] pc_out, adder_out,instr_mem_out, rd1_out, rd2_out, signExtend_out, mux_alu_out, aluData_out, mux_writeData_out, shift_adder_out, aluBranch_out, readData_out, mux_mux_out, muxPC_out;
  inout RegDst_out, jump_out, branch_out, MemRead_out, MemtoReg_out, MemWrite_out, ALUSrc_out, RegWrite_out, gate_out, zero_out;
  inout[4:0] mux_register_out;
  inout[27:0] shift_mux_out;
  inout[1:0] ALUOp_out;
  inout[2:0] ALU_control_out;
  input clk, R;
  
  
  registers registri(.rs(instr_mem_out[25:21]), .rt(instr_mem_out[20:16]), .write_r(mux_register_out), .write_d(mux_writeData_out), .read_data1(rd1_out),
.read_data2(rd2_out), .RegWrite(RegWrite_out), .clk(clk));
                 
  instruction_mem Instruction_Memory(.read_address(pc_out), .instruction(instr_mem_out));
                 
  Control modControl(.control_in(instr_mem_out[31:26]), .regDst(RegDst_out), .Jump(jump_out), .Branch(branch_out), .MemRead(MemRead_out),.MemToReg(MemtoReg_out), .ALUOp(ALUOp_out), .MemWrite(MemWrite_out), .ALUSrc(ALUSrc_out), .RegWrite(RegWrite_out));
  
  program_counter PC(.in(muxPC_out), .out(pc_out), .R(R), .clk(clk));
                                 
  shift_left shift2left(.in(signExtend_out), .out(shift_adder_out));  
                                 
  shift_left_26_28 shift26_28left(.ins(instr_mem_out[25:0]), .outs(shift_mux_out));
                   
  ALUControl aluControl(.instruction(instr_mem_out[5:0]), .ALUOp(ALUOp_out), .ALU_out(ALU_control_out));
                   
  ALU alu(.x(rd1_out), .y(rd2_out), .alu_control(ALU_control_out), .alu_out(aluData_out), .zero_out(zero_out));
                   
  DataMemory MemoryData(.alu_result(aluData_out), .rd2(rd2_out), .mem_read(MemRead_out), .mem_write(MemWrite_out), .read_data(readData_out), .clk(clk));
                   
  mux_32 multiplexor_branch(.x(aluBranch_out), .y(adder_out), .S(zero_out && branch_out), .out(mux_mux_out));
                                  
  mux_32 multiplexor_PC(.x({shift_mux_out, adder_out[31:28]}), .y(mux_mux_out), .S(jump_out), .out(muxPC_out));
                                  
  mux_32 multiplexor_ALU(.x(signExtend_out), .y(rd2_out), .S(ALUSrc_out), .out(mux_alu_out));
                             
  mux_32 multiplexor_DataMemory(.x(readData_out), .y(aluData_out), .S(MemtoReg_out), .out(mux_writeData_out));
                                  
  mux_5 multiplexor_5(.x(instr_mem_out[20:16]), .y(instr_mem_out[15:11]), .S(RegDst_out), .out(mux_register_out));
                         
  signExtend sign16_32extend(.in(instr_mem_out[15:0]), .out(signExtend_out));  
                         
  adderCounter adderProgramCounter(.pc_out(pc_out), .out(adder_out));
                         
  adder adder2var_out(.in1(adder_out), .in2(shift_adder_out), .out(aluBranch_out));                                                                                                                                           
endmodule
  
                 
              
