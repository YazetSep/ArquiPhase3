//////////// Instrcution Fetch (IF) ///////////

/* IF/ID */

//////////// Instruction Decode (ID) //////////

/* ID/EXE */

module ID_EXE_pipeline(output reg [31:0] EXE_In, EXE_A, EXE_B, output reg [11:0] EXE_immed, output reg [4:0] EXE_Opcode, 
output reg [3:0] EXE_Rd_num, output reg [2:0] EXE_I_cmd, output reg EXE_S, input [31:0] ID_A, ID_B, instruction, input clk);
    always @ (posedge clk)
    begin
        EXE_In <= ID_A;
        EXE_A <= ID_A;
        EXE_B <= ID_B;
        EXE_immed <= instruction[11:0];
        EXE_Opcode <= instruction[24:20];
        EXE_Rd_num <= instruction[15:12];
        EXE_I_cmd <= instruction[27:25];
        EXE_S <= instruction[20];
    end
endmodule 

/////////////// Execution (EX) ///////////////

module shifter_sign_extender(output reg [31:0] Out, output reg LS, input [31:0] Rm, 
input[31:0] Rn, input [11:0] I, input[4:0] Opcode, input [2:0] I_cmd);
  reg temp1;
  always @ (Rm, Rn, I, Opcode, I_cmd)
  begin
    LS = 1'b0;
    case (I_cmd)
      3'b001 : begin // 32-bit Immediate Shifter Operand : Rotation of immediate
      Out = 32'b0 + I[7:0];
      
      repeat (I[11:8] * 2)
        begin
          temp1 = Out[0];
          Out = Out >> 1;
          Out[31] = temp1;
        end
      end
      3'b000 : begin // Shift by immediate
        Out = Rm;
        case (I[6:5])
        2'b00 : Out = Rm << I[11:7]; // LSL: Logical Shift Left 
        2'b01 : Out = Rm >> I[11:7]; // LSR: Logical Shift Right
        2'b10 : begin repeat (I[11:7])     // ASR: Arithmetic Shift Right
            begin
              Out = Out >> 1;
              Out[31] = 1'b1;
            end
          end 
        2'b11 : begin repeat (I[11:7])     // ROR: Rotate Right
            begin
              temp1 = Out[0];
              Out = Out >> 1;
              Out[31] = temp1;
            end
          end 
        endcase
      end
      3'b010 : begin
        LS = 1'b1;
         if (Opcode[3]) Out = Rn + I; 
         else Out = Rn - I;  // Load/Store Immediate offset
      end
      3'b011 : begin
        LS = 1'b1;
        if (Opcode[3]) Out = Rn + Rm; 
        else Out = Rn - Rm;  // Load/Store Register offset
      end
      default : Out = Rm;
    endcase
  end
endmodule

module ALU_mux(output reg [31:0] Out, input [31:0] B, immed, input shift_imm);
  always @ (shift_imm, B, immed)
      if (shift_imm) Out = immed;
      else Out = B;
endmodule

module ALU_component (output reg [31:0] Out, output reg N, Z, C, V, // CC = N, Z, C, V
input [31:0] A, B, input [3:0] ALU_op, input Ci); 
  reg temp;
  reg [31:0]tempOut; // For cases where output doesn't matter
  always @ (ALU_op,A,B)
  begin
    N = 1'b0; Z = 1'b0; C = 1'b0; V = 1'b0; temp = 1'b0;
    case (ALU_op)
      4'b0000 : Out = A & B;                    // AND: Logical AND
      4'b0001 : {temp, Out} = A ^ B;            // EOR: Logical Exclusive OR
      4'b0010 : {temp, Out} = A - B;            // SUB: Subtract
      4'b0011 : {temp, Out} = B - A;            // RSB: Reverse Subtract
      4'b0100 : {temp, Out} = A + B;            // ADD: Add
      4'b0101 : {temp, Out} = A + B + Ci;       // ADC: Add	with Carry
      4'b0110 : {temp, Out} = A - B - ~(Ci);    // SBC: Subtract with Carry
      4'b0111 : {temp, Out} = B - A - ~(Ci);    // RSC: Reverse Subtract with Carry
      4'b1000 : tempOut = A & B;                // TST: Test
      4'b1001 : tempOut = A ^ B;                // TEQ: Test Equivalence
      4'b1010 : tempOut = A - B;                // CMP: Compare
      4'b1011 : tempOut = A + B;                // CMN: Compare Negated
      4'b1100 : Out = A | B;                    // ORR: Logical Or
      4'b1101 : Out = B;                        // MOV: Move
      4'b1110 : Out = A & ~(B);                 // BIC: Bit Clear
      4'b1111 : Out = ~(B);                     // MVN: Move Not
    endcase  

    C = temp;
    N = Out[31];
    Z = ~(Out && Out);
    // Considers Overflow for addition operations
    if ((ALU_op === 4'b0100) | (ALU_op === 4'b0101) | (ALU_op === 4'b1011)) V = (Out[31] && ~(A[31] & B[31]));
    // Considers Overflow for subtraction
    if ((ALU_op === 4'b0010) | (ALU_op === 4'b0011) | (ALU_op === 4'b0110) | (ALU_op === 4'b0111) | (ALU_op === 4'b1010)) V = ((A[31] == ~(B[31])) && (Out[31] == B[31]));

    if ((ALU_op >= 4'b1000) & (ALU_op <= 4'b1011)) begin
      N = tempOut[31];
      Z = tempOut && 32'b0;
      Out = 32'b0;
      end
  end
    
endmodule

module ALUvsSSE_mux(output reg [31:0] Out, input [31:0] ALU_Out, SSE_Out, input LS);
  always @ (LS, ALU_Out, SSE_Out)
      if (LS) Out = SSE_Out;
      else Out = ALU_Out;
endmodule

module status_register(output reg N, Z, C, V, input Ni, Zi, Ci, Vi, S);
  always @ (Ni, Zi, Ci, Vi, S)
    begin 
      N = 1'b0; Z = 1'b0; C = 1'b0; V = 1'b0;
      if (S) begin
        N = Ni; Z = Zi; C = Ci; V = Vi;
      end
    end
endmodule

module condition_handler(output reg out, input[3:0] cond, input B_instr, N, Z, C, V);
  always @ (cond, B_instr, N, Z, C, V)
    begin
      out = 1'b0;
      if(B_instr) begin
        case(cond)
          4'b0000 : if(Z) out = 1'b1;               // Equal
          4'b0001 : if(~(Z)) out = 1'b1;            // Not equal
          4'b0010 : if(C) out = 1'b1;               // Unsigned	higher or same
          4'b0011 : if(~(C)) out = 1'b1;            // Unsigned lower
          4'b0100 : if(N) out = 1'b1;               // Minus
          4'b0101 : if(~(N)) out = 1'b1;            // Positive or Zero
          4'b0110 : if(V) out = 1'b1;               // Overflow
          4'b0111 : if(~(V)) out = 1'b1;            // No overflow
          4'b1000 : if(C & ~(Z)) out = 1'b1;        // Unsigned higher
          4'b1001 : if(~(C) | Z) out = 1'b1;        // Unsigned lower or same
          4'b1010 : if(N == V) out = 1'b1;          // Greater or equal
          4'b1011 : if(N != V) out = 1'b1;          // Less than
          4'b1100 : if(~(Z) & (N == V)) out = 1'b1; // Greater than
          4'b1101 : if(Z & (N != V)) out = 1'b1;    // Less than or equal
          4'b1110 : out = 1'b1;                     // Always
        endcase
      end
    end
endmodule

/* EXE/MEM */

module EXE_MEM_Pipeline(output reg [31:0]DataMemIn, output reg [31:0]AddressDataOut, output reg [3:0]Rd_Out, output reg [1:0]DataSizeOut, output reg LoadInst, output reg RF_EN, input [1:0]DataSizeIn, input [31:0]ALU_Out, input N, Z, C, V, input [31:0]DataIn, input [3:0]Rd_In, input LI, input RF_Enable)
    always @(posedge clk)
    begin
        DataMemIn <= DataIn;
        AddressDataOut <= ALU_Out;
        LoadInst <= LI;
        RF_EN <= RF_Enable;
        DataSizeOut <= DataSizeIn;
        Rd_Out <= Rd_In;
    end
    
endmodule

///////////////// Memory (MEM) ///////////////

/* MEM/WB */

/////////////// Write-Back (WB) //////////////


//######### Miscellaneous ##########//

module control_unit(output reg [3:0] ID_ALU_OP, output reg [1:0] data_size, 
output reg ID_shift_imm, ID_load_instr, ID_RF_enable, ID_B_instr, RW, input [31:0] instruction);
    always @ (instruction)
    begin
        ID_ALU_OP = instruction[24:21];
        ID_shift_imm = instruction[25];
        ID_load_instr = ((instruction[27:25] === 3'b010) | (instruction[27:25] === 3'b011)) & instruction[20];
        ID_RF_enable = (instruction[15:12] == 1'b1);     // If there is a destination register
        ID_B_instr = (instruction[27:25] === 3'b101);
        // Verify
        RW = ((ID_load_instr) | ~(ID_RF_enable));
        // TODO
        data_size = 2'b00;
        //
    end
endmodule


//#########//
// TESTING //
//#########//

module test_CPU;
endmodule