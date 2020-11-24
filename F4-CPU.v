//////////// Instrcution Fetch (IF) ///////////
module inst_fetch(output reg [31:0]DataOut, output reg [31:0]NextPC, output reg [31:0]PC_In, input [31:0]TargetAddress, input [31:0]PC, input condition_handler_in, input EN, clk);
  wire [31:0]DOut;
  wire [31:0]MOut;
  reg [31:0]InstTemp;
  
  ram256x32_inst ram1 (DOut, 1'b1, PC);
  
  mux_2x1_32Bit mux (MOut, condition_handler_in, NextPC, TargetAddress);
  
  initial DataOut = 32'b0;
  
  always @(posedge EN)
  begin
    //NextPC <= 4;
    //#1 $display("NPC: %b", NextPC);
    InstTemp[31:24] = ram1.Mem[0]; //Always in Read Mode
    InstTemp[23:16] = ram1.Mem[1];
    InstTemp[15:8] = ram1.Mem[2];           
    InstTemp[7:0] = ram1.Mem[3];
    // Checks for invalid instructions
    if (InstTemp[27:25] === 3'b0 && InstTemp[4] || // Ignore register shift
      InstTemp[27:26] === 3'b01 && (~InstTemp[24] | InstTemp[21]) || // LoSt: Imm and register offset only 
      InstTemp[27:25] === 3'b100 || // Ignore LoSt multiple
      InstTemp[27:26] === 3'b11) begin // Ignore Coprocessor and SWI instructions
      DataOut <= 32'b0; // Insert NOP if the instruction is invalid
      end
    else DataOut <= InstTemp;
    NextPC <= 32'b100;
    PC_In <= 32'b100;
    //#1 $display("PC IN: %b", PC_In);e11100000100000100101000000000101
  end
  
  always @(TargetAddress, PC, condition_handler_in, posedge clk)
    begin
      NextPC <= PC + 4;
      // Checks for invalid instructions
      if (DOut[27:25] === 3'b0 && DOut[4] || // Ignore register shift
      DOut[27:26] === 3'b01 && (~DOut[24] | DOut[21]) || // LoSt: Imm and register offset only 
      DOut[27:25] === 3'b100 || // Ignore LoSt multiple
      DOut[27:26] === 3'b11) begin // Ignore Coprocessor and SWI instructions
          DataOut <= 32'b0; // Insert NOP if the instruction is invalid
      end
      else DataOut <= DOut;
      PC_In <= MOut;
    end
endmodule

module mux_2x1_32Bit (output reg [31:0] Y, input S, input [31:0] A, B);
  always @ (*)
  begin
    Y = A;
    if (S) Y = B;  
  end
endmodule

module ram256x32_inst(output reg [31:0]DataOut, input Enable, input [31:0]PC);

  reg [7:0]Mem[0:1023];
  reg [7:0]Address = 8'b00000000;
  
  always @(*)
    if(Enable)
    begin
      DataOut = 0;
      Address = PC[7:0]; 
      DataOut[31:24] = Mem[Address]; //Always in Read Mode
      DataOut[23:16] = Mem[Address+1];
      DataOut[15:8] = Mem[Address+2];
      DataOut[7:0] = Mem[Address+3];
    end

endmodule

/* IF/ID */
module IFID (output reg [31:0] PC_out, instr_out, input [31:0] PC_in, instr_in, input clk, CH_reset, LE, totalReset);

  always @ (posedge totalReset) begin
    PC_out <= 0;
    instr_out <= 0;
  end

  always @(posedge clk) begin
    PC_out <= PC_in;
    instr_out <= instr_in;  
    if (CH_reset)
      instr_out <= 32'b0; //Control hazard handling reset
    else if (~(LE)) begin //LE is used to stall when load hazard is asserted from the Haz/Forw Unit
      PC_out <= 32'b0;
      instr_out <= 32'b0;
    end
end
endmodule

//////////// Instruction Decode (ID) //////////

module x4_Sign_Extender(output reg [31:0] extended, input [23:0] input_instr);
  always @ (*) begin
    if (input_instr[23]) extended = {{8{1'b1}}, input_instr};
    else extended = {{8{1'b0}}, input_instr};
    extended = extended * 4;
  end
endmodule

module Adder (output reg [31:0] DataOut, input [31:0] extended_instr, NextPC);
  always @ (extended_instr, NextPC)
    DataOut = extended_instr + NextPC;
endmodule

//Multiplexers for Forwarding
module mux_4x1_32Bit (output reg [31:0] Y, input [1:0] S, input [31:0] A, B, C, D);
  always @ (S, A, B, C, D)
    case (S)
    2'b00: Y = A;
    2'b01: Y = B;
    2'b10: Y = C;
    2'b11: Y = D;
    endcase
    
endmodule

//--------------Register File-----------------------
module RegisterFile (output [31:0] PuertoA, PuertoB, PC_out, PD, input [31:0] PW, PC_in, input [3:0] RW, RA, RB, RD, input LE, PCLd, Clk, reset);
  //Outputs: Puertos A, B y PC_out
  //Inputs: Puerto de Entrada (PW), RW y LE (BinaryDecoder Selector (Registro Destino) y "load"), RA y RB  (Selectors de multiplexers a la salida AKA Source Registers), y Clk
  //Wires
  wire [15:0] E;
  wire [31:0] Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, Q11, Q12, Q13, Q14, Q15;
  wire [31:0] mux_PCOut;
  wire reg15Ld;

  //Instanciando módulos:
  binaryDecoder bdecoder (E, RW, LE);
  //Multiplexer for PuertoA
  mux_16x1_32Bit mux_16x1A (PuertoA, RA, Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9,
                  Q10, Q11, Q12, Q13, Q14, Q15);
  //Multiplexer for PuertoB
  mux_16x1_32Bit mux_16x1B (PuertoB, RB, Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9,
                  Q10, Q11, Q12, Q13, Q14, Q15);
  //Multiplexer for PuertoD (Used for store instructions)
  mux_16x1_32Bit mux_16x1D (PD, RD, Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9,
                  Q10, Q11, Q12, Q13, Q14, Q15);
  //Multiplexer to control PC_in (Priority writing data instead of writing new PC value)
  mux_2x1_32Bit mux_2x1PC_in (mux_PCOut, E[15], PC_in, PW);

  //OR gate to activate load of Register15/PC with Binary Decoder E[15] port or PCLd port
  or (reg15Ld, E[15], PCLd);

  //Registers 0 to 15
  register_32bit R0 (Q0, PW, Clk, E[0], reset);
  register_32bit R1 (Q1, PW, Clk, E[1], reset);
  register_32bit R2 (Q2, PW, Clk, E[2], reset);
  register_32bit R3 (Q3, PW, Clk, E[3], reset);
  register_32bit R4 (Q4, PW, Clk, E[4], reset);
  register_32bit R5 (Q5, PW, Clk, E[5], reset);
  register_32bit R6 (Q6, PW, Clk, E[6], reset);
  register_32bit R7 (Q7, PW, Clk, E[7], reset);
  register_32bit R8 (Q8, PW, Clk, E[8], reset);
  register_32bit R9 (Q9, PW, Clk, E[9], reset);
  register_32bit R10 (Q10, PW, Clk, E[10], reset);
  register_32bit R11 (Q11, PW, Clk, E[11], reset);
  register_32bit R12 (Q12, PW, Clk, E[12], reset);
  register_32bit R13 (Q13, PW, Clk, E[13], reset);
  register_32bit R14 (Q14, PW, Clk, E[14], reset);
  register_32bit R15 (Q15, mux_PCOut, Clk, reg15Ld, reset);//Special Register also functions as Program Counter (PC)
  assign PC_out = Q15;
  
endmodule

module mux_16x1_32Bit (output reg [31:0] Y, input [3:0] S, 
input [31:0] R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15);
  always @ (S, R0, R1, R2, R3, R4, R5, R6, R7, R8, R9,
                  R10, R11, R12, R13, R14, R15)
  case (S)
  4'b0000: Y = R0;
  4'b0001: Y = R1;
  4'b0010: Y = R2;
  4'b0011: Y = R3;
  4'b0100: Y = R4;
  4'b0101: Y = R5;
  4'b0110: Y = R6;
  4'b0111: Y = R7;
  4'b1000: Y = R8;
  4'b1001: Y = R9;
  4'b1010: Y = R10;
  4'b1011: Y = R11;
  4'b1100: Y = R12;
  4'b1101: Y = R13;
  4'b1110: Y = R14;
  4'b1111: Y = R15;
  endcase
endmodule

module mux_2x1_OneBit (output reg Y, input S, A, B);
  always @ (S, A, B)
    if (S) Y = B;
    else Y = A;
endmodule

module binaryDecoder (output reg [15:0] E, input [3:0] C, input RF);
  always @ (C, RF)
    if (!RF) E = 0;
    else begin
      E = 1;
      case (C)
      4'b0001: E = E << 1;
      4'b0010: E = E << 2;
      4'b0011: E = E << 3;
      4'b0100: E = E << 4;
      4'b0101: E = E << 5;
      4'b0110: E = E << 6;
      4'b0111: E = E << 7;
      4'b1000: E = E << 8;
      4'b1001: E = E << 9;
      4'b1010: E = E << 10;
      4'b1011: E = E << 11;
      4'b1100: E = E << 12;
      4'b1101: E = E << 13;
      4'b1110: E = E << 14;
      4'b1111: E = E << 15;  
      default: E = 1;
      endcase
    end
endmodule

module register_32bit (output reg [31:0] Q, input [31:0] D, input Clk, Ld, reset);
  always @ (posedge reset, posedge Clk) begin
    if (reset) Q = 0;
    else if(Ld) Q = D;
  end
endmodule

/* ID/EXE */

module ID_EXE_pipeline(output reg [31:0] EXE_In, EXE_A, EXE_B, output reg [11:0] EXE_immed, output reg [4:0] EXE_Opcode, output reg [3:0] EXE_Rd_num, EXE_ALU_OP, output reg [2:0] EXE_I_cmd, 
output reg EXE_S, EXE_shift_imm, EXE_load_instr, EXE_RF_enable, EXE_RW, output reg [1:0] EXE_DataSize ,input [31:0] ID_A, ID_B, ID_D, instruction, input[3:0] ID_ALU_OP, input ID_shift_imm, ID_load_instr, ID_RF_enable, clk, input [1:0] ID_DataSize, input ID_RW, I_EN, reset);
  always @(posedge reset) begin
  	EXE_In <= 0;
    EXE_A <= 0;
    EXE_B <= 0;
    EXE_immed <= 0;
    EXE_Opcode <= 0;
    EXE_Rd_num <= 0;
    EXE_I_cmd <= 0;
    EXE_S <= 0;
    EXE_shift_imm <= 0;
    EXE_ALU_OP <= 0;
    EXE_load_instr <= 0;
    EXE_RF_enable <= 0;
    EXE_RW <= 0;
    EXE_DataSize = 0;
  end
  
  always @ (posedge clk)
  begin
    // If the condition of the instruction passes, the instruction moves onto the next stage
    if (I_EN) begin
      EXE_In <= ID_D;
      EXE_A <= ID_A;
      EXE_B <= ID_B;
      EXE_immed <= instruction[11:0];
      EXE_Opcode <= instruction[24:20];
      EXE_Rd_num <= instruction[15:12];
      EXE_I_cmd <= instruction[27:25];
    	EXE_S <= instruction[20];
      EXE_shift_imm <= ID_shift_imm;
      EXE_ALU_OP <= ID_ALU_OP;
      EXE_load_instr <= ID_load_instr;
      EXE_RF_enable <= ID_RF_enable;
    	EXE_RW <= ID_RW;
   		EXE_DataSize = ID_DataSize;
    end
    // Otherwise, the instruction should not be processed, so a NOP is passed onto EXE
    else begin
      EXE_In <= 0;
      EXE_A <= 0;
      EXE_B <= 0;
      EXE_immed <= 0;
      EXE_Opcode <= 0;
      EXE_Rd_num <= 0;
      EXE_I_cmd <= 0;
      EXE_S <= 0;
      EXE_shift_imm <= 0;
      EXE_ALU_OP <= 0;
      EXE_load_instr <= 0;
      EXE_RF_enable <= 0;
    	EXE_RW <= 0;
    	EXE_DataSize = 0;
    end
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
      3'b010 : begin // Load/Store Immediate offset
        LS = 1'b1;
        if (Opcode[3]) Out = Rn + I; 
        else Out = Rn - I;  
      end
      3'b011 : begin // Load/Store Register offset
        LS = 1'b1;
        if (Opcode[3]) Out = Rn + Rm; 
        else Out = Rn - Rm; 
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

module condition_handler(output reg B_out, I_EN, input[3:0] cond, input B_instr, N, Z, C, V);
  always @ (cond, B_instr, N, Z, C, V)
    begin
      I_EN = 1'b0;
      case(cond)
        4'b0000 : if(Z) I_EN = 1'b1;               // Equal
        4'b0001 : if(~(Z)) I_EN = 1'b1;            // Not equal
        4'b0010 : if(C) I_EN = 1'b1;               // Unsigned	higher or same
        4'b0011 : if(~(C)) I_EN = 1'b1;            // Unsigned lower
        4'b0100 : if(N) I_EN = 1'b1;               // Minus
        4'b0101 : if(~(N)) I_EN = 1'b1;            // Positive or Zero
        4'b0110 : if(V) I_EN = 1'b1;               // Overflow
        4'b0111 : if(~(V)) I_EN = 1'b1;            // No overflow
        4'b1000 : if(C & ~(Z)) I_EN = 1'b1;        // Unsigned higher
        4'b1001 : if(~(C) | Z) I_EN = 1'b1;        // Unsigned lower or same
        4'b1010 : if(N == V) I_EN = 1'b1;          // Greater or equal
        4'b1011 : if(N != V) I_EN = 1'b1;          // Less than
        4'b1100 : if(~(Z) & (N == V)) I_EN = 1'b1; // Greater than
        4'b1101 : if(Z & (N != V)) I_EN = 1'b1;    // Less than or equal
        default : I_EN = 1'b1;                     // Always
      endcase
      B_out = B_instr & I_EN;
    end
endmodule

/* EXE/MEM */

module EXE_MEM_Pipeline(output reg [31:0] DataMemIn, AddressDataOut, output reg [3:0]Rd_Out, output reg [1:0]DataSizeOut, output reg LoadInst, RF_EN, MEM_LS, MEM_RW,
input [31:0]ALU_Out, DataIn, input [3:0] Rd_In, input [1:0]DataSizeIn, input LI, RF_Enable, EXE_LS, N, Z, C, V, clk, reset, EXE_RW);
    
  always @(posedge reset) begin
    DataMemIn <= 0;
    AddressDataOut <= 0;
    LoadInst <= 0;
    RF_EN <= 0;
    DataSizeOut <= 0;
    Rd_Out <= 0;
    MEM_LS <= 0;
  	MEM_RW <= 0;
  end
  
  always @(posedge clk)
  begin
    DataMemIn <= DataIn;
    AddressDataOut <= ALU_Out;
    LoadInst <= LI;
    RF_EN <= RF_Enable;
    DataSizeOut <= DataSizeIn;
    Rd_Out <= Rd_In;
    MEM_LS <= EXE_LS;
  	MEM_RW <= EXE_RW;
  end
    
endmodule

///////////////// Memory (MEM) ///////////////

module memory(output reg [31:0]DataOut, AddressOut, MuxOut, input [31:0]DataIn, AddressIn, input RW, input [1:0]DataSize, input MuxController, Enable);

  wire [31:0]DOut;
  wire [31:0]AOut;
  wire [31:0]MOut;

  ram256x32_data dataRam (DOut, Enable, RW, DataIn, AddressIn, DataSize);
  mux_2x1_32Bit mux (MOut, MuxController, AddressIn, DOut);
  
  initial begin : named_block8
    integer i;
    DataOut = 0;
    AddressOut = 0;
    MuxOut = 0;
    for (i = 0; i < 1023; i++) begin : named_block7
      dataRam.Mem[i] = 8'b0;
    end 
  end

  always @(*)
    begin
      DataOut = DOut;
      AddressOut = AddressIn;
      MuxOut = MOut;
    end
endmodule

module ram256x32_data(output reg [31:0]DataOut, input Enable, ReadWrite, input[31:0]DataIn, input [31:0]Address, input [1:0]DataSize);

  reg [7:0]Mem[0:1023];
  reg [31:0]DataTemp;
  always @(*)
  begin
    if(Enable)
      case (DataSize)
        //Byte
        2'b00 : if (ReadWrite) begin : named_block1 //Read - Load 
            DataOut = 0;
            DataOut[7:0] = Mem[Address];
          end 
          else //Write - Store
            begin : named_block2//Write - Store
                Mem[Address] = DataIn[7:0];
            end
        //Half-Wrd
        2'b01 : if (ReadWrite) begin : named_block3 //Read - Load
            DataOut = 0;
            DataOut[15:8] = Mem[Address+2];
            DataOut[7:0] = Mem[Address+3];
          end 
          else 
            begin : named_block4 //Write - Store
                Mem[Address+2] = DataIn[15:8];
                Mem[Address+3] = DataIn[7:0];
            end
        //Word
        2'b10 : if (ReadWrite) begin //Read - Load
            DataOut = 0;
            DataOut[31:24] = Mem[Address];
            DataOut[23:16] = Mem[Address+1];
            DataOut[15:8] = Mem[Address+2];
            DataOut[7:0] = Mem[Address+3]; 
          end 
          else 
            begin //Write - Store
              DataOut = 0;
              Mem[Address] = DataIn[31:24];
              Mem[Address+1] = DataIn[23:16];
              Mem[Address+2] = DataIn[15:8];
              Mem[Address+3] = DataIn[7:0];
            end
        //Double Word
        2'b11 : if (ReadWrite) 
          begin //Read - Load
            DataOut = 0;
            DataOut[31:24] = Mem[Address];
            DataOut[23:16] = Mem[Address+1];
            DataOut[15:8] = Mem[Address+2];
            DataOut[7:0] = Mem[Address+3];
            #1;
            DataOut = 0;
            DataOut[31:24] = Mem[Address+4];
            DataOut[23:16] = Mem[Address+5];
            DataOut[15:8] = Mem[Address+6];
            DataOut[7:0] = Mem[Address+7];
          end 
          else 
            begin //Write - Store
              DataOut = 0;
              Mem[Address] = DataIn[31:24];
              Mem[Address+1] = DataIn[23:16];
              Mem[Address+2] = DataIn[15:8];
              Mem[Address+3] = DataIn[7:0];
              #1;
              DataOut = 0;
              Mem[Address+4] = DataIn[31:24];
              Mem[Address+5] = DataIn[23:16];
              Mem[Address+6] = DataIn[15:8];
              Mem[Address+7] = DataIn[7:0];
            end
      endcase
    else begin
        DataOut = 0;
    
    end
  end
endmodule
/* MEM/WB */

module MEM_WB_Pipeline(output reg [31:0]DataOut, output reg [31:0]NonLoadDataOut, output reg [3:0]Rd_Out, output reg LoadInst, output reg RF_EN, input [31:0]MuxLoadDataIn, input [31:0]MuxNonLoadDataIn, input [3:0]Rd_In, input LI, RF_Enable, clk, reset);

  always @(posedge reset) begin
  DataOut <= 0;
    NonLoadDataOut <= 0;
    Rd_Out <= 0;
    LoadInst <= 0;
    RF_EN <= 0;
  end

  always @(posedge clk)
  begin
    DataOut <= MuxLoadDataIn;
    NonLoadDataOut <= MuxNonLoadDataIn;
    Rd_Out <= Rd_In;
    LoadInst <= LI;
    RF_EN <= RF_Enable;

end

endmodule

/////////////// Write-Back (WB) //////////////

module writeback (output reg [31:0]DataOut, input [31:0]AddressIn, input[31:0]DataIn, input LoadInst);
  wire [31:0]DOut;

  mux_2x1_32Bit mux (DOut, LoadInst, AddressIn, DataIn);
  always @(*)
  begin
    DataOut = DOut;

  end

endmodule

//######### Miscellaneous ##########//

module control_unit(output reg [3:0] ID_ALU_OP, output reg [1:0] data_size, 
output reg ID_shift_imm, ID_load_instr, ID_RF_enable, ID_B_instr, ID_RW, input [31:0] instruction);
    always @ (instruction)
    begin
      ID_B_instr = (instruction[27:25] === 3'b101);
      // LS instructions default as ADD or SUB
      if (instruction[27:26] === 3'b01) ID_ALU_OP = instruction[23]? 4'b0100 : 4'b0010;
      // Otherwise use the ALU_op bits if not a branch instruction
      else ID_ALU_OP = ID_B_instr? 4'b0 : instruction[24:21];

      ID_shift_imm = instruction[27:26] === 3'b00 & (instruction[25] | (~instruction[25] && instruction[11:4])) | (instruction[27:25] === 3'b010);

      ID_load_instr = (instruction[27:26] === 3'b01) & instruction[20];
			// To have a destination register, the instruction CAN'T be:
      ID_RF_enable = ~(instruction[27:26] === 3'b00 && (instruction[24:23] === 2'b10)) & // -A CMP, CMN, TST, or TEQ Data processing instruction
      	~(ID_B_instr) & // -A branch instruction
      	~((instruction[27:26] === 3'b01) & ~(instruction[20])); // -A Store instruction
      
      ID_RW = instruction[20];

      data_size = 2'b00;
      if(instruction[27:25] === 3'b011 || instruction[27:25] === 3'b010)
        data_size = instruction[22]? 2'b00 : 2'b10;
    end
endmodule

module CU_Mux (output reg [3:0] ID_ALU_OP, output reg ID_shift_imm, ID_load_instr, ID_RF_enable, ID_RW, output reg [1:0]ID_DataSize, input [3:0] ID_ALU_OP_CU, input ID_shift_imm_CU, ID_load_instr_CU, ID_RF_enable_CU, S, ID_RW_CU, input [1:0]ID_DataSize_CU);
  always @ (*)
    begin
      ID_ALU_OP = ID_ALU_OP_CU;
      ID_shift_imm = ID_shift_imm_CU;
      ID_load_instr = ID_load_instr_CU;
      ID_RF_enable = ID_RF_enable_CU;
      ID_RW = ID_RW_CU;
      ID_DataSize = ID_DataSize_CU;
      
      if (~(S)) begin
      	ID_ALU_OP = 4'b0;
      	ID_shift_imm = 1'b0;
      	ID_load_instr = 1'b0;
      	ID_RF_enable = 1'b0;
        ID_RW = 1'b0;
        ID_DataSize = 1'b0;
    	end
    end
endmodule

module HazForwUnit (output reg [1:0] mux_S_A, mux_S_B, mux_S_D, output reg Nop_insertion_S, IFID_enable, PC_enable,
input [3:0] EX_Rd, MEM_Rd, WB_Rd, ID_Rn, ID_Rm, ID_Rd, input EX_RF_enable, MEM_RF_enable, WB_RF_enable, EX_load_instr, ID_shift_imm);
    /*Mux A and Mux B Cheat Sheet:
      00 -> PA/PB Original Register File Outputs
      01 -> [EX_Rd]
      10 -> [MEM_Rd]
      11 -> [WB_Rd]
      Note: The multiplexers are mirrored vertically. Top port of Mux A is selected with 11 while top port of Mux B is selected with 00. (Referring to diagram)
    */
    always @ (*) begin
        //TODO Special Forwarding Cases: Forwarding is only allowed from the stage closest to ID
    
        //Data Forwarding Detection and Handling
        if (EX_RF_enable && (ID_Rn === EX_Rd)) //EX forwarding
          mux_S_A <= 2'b01; //Forwarding [EX_Rd] to ID
        else if (MEM_RF_enable && (ID_Rn === MEM_Rd)) //MEM forwarding
          mux_S_A <= 2'b10; //Forwarding [MEM_Rd] to ID
        else if (WB_RF_enable && (ID_Rn === WB_Rd)) //WB forwarding
          mux_S_A <= 2'b11; //Forwarding [WB_Rd] to ID
        else mux_S_A <= 2'b00; //Not forwarding (passing PA from the register file)
    
        if (EX_RF_enable && (ID_Rm === EX_Rd)) //EX forwarding
          mux_S_B <= 2'b01; //Forwarding [EX_Rd] to ID
        else if (MEM_RF_enable && (ID_Rm === MEM_Rd)) //MEM forwarding
          mux_S_B <= 2'b10; //Forwarding [MEM_Rd] to ID
        else if (WB_RF_enable && (ID_Rm === WB_Rd)) //WB forwarding
          mux_S_B <= 2'b11; //Forwarding [WB_Rd] to ID
        else mux_S_B <= 2'b00; //Not forwarding (passing PB from the register file)

        if (EX_RF_enable && (ID_Rd === EX_Rd)) //EX forwarding
          mux_S_D <= 2'b01; //Forwarding [EX_Rd] to ID
        else if (MEM_RF_enable && (ID_Rd === MEM_Rd)) //MEM forwarding
          mux_S_D <= 2'b10; //Forwarding [MEM_Rd] to ID
          else if (WB_RF_enable && (ID_Rd === WB_Rd)) //WB forwarding
          mux_S_D <= 2'b11; //Forwarding [WB_Rd] to ID
        else mux_S_D <= 2'b00; //Not forwarding (passing PD from the register file)
    
        //Detecting and Handling Load Hazard
        if (EX_load_instr && ((ID_Rn === EX_Rd) || ((ID_Rm === EX_Rd) && !ID_shift_imm))) begin
          //Hazard asserted
          Nop_insertion_S <= 1'b0; //Forward control signals corresponding to a nop instruction
          IFID_enable <= 1'b0; //Disable IF/ID pipeline register from loading
          PC_enable <= 1'b0; //Disable load enable of the program counter
        end
        else begin
          //Hazard not asserted
          Nop_insertion_S <= 1'b1;
          IFID_enable <= 1'b1;
          PC_enable <= 1'b1;
        end
    end
endmodule


//#########//
// TESTING //
//#########//

module test_CPU;
  //reg [31:0] instruction; 
  reg clk;
  reg TR = 1'b0;
  integer track;
    ////// WIRES //////
   // IF wires
  wire [31:0] IF_DataOut, IF_NextPC, IF_PC_In;

  	// ID wires
  //reg [31:0] ID_PC_out; 
  wire [31:0] ID_PC_out;
  wire [31:0]RegPC_Out;
  wire [31:0]ID_instr_out; // IF/ID Pipeline
  wire [31:0] PA, PB, PD; //Register File
  wire [31:0] extended; //X4 Sign Extender
  wire [31:0] ID_TargetAdress; //Adder output (extended + ID_PC_out)
  wire [31:0] ID_mux_A_out, ID_mux_B_out, ID_mux_D_out; //Multiplexers used for forwarding
  // Control Unit
  wire [3:0] ID_ALU_OP_CU;
  wire [1:0] ID_data_size_CU, ID_data_size;
  wire ID_shift_imm_CU, ID_load_instr_CU, ID_RF_enable_CU, ID_B_instr, ID_RW, ID_RW_CU;
  // Control Unit Mux
  wire [3:0] ID_ALU_OP;
  wire ID_shift_imm, ID_load_instr, ID_RF_enable;
         
  	// EXE wires
  // ID/EXE Pipeline Register
  wire [31:0] EXE_In, EXE_A, EXE_B;
  wire [11:0] EXE_immed;
  wire [4:0] EXE_Opcode;
  wire [3:0] EXE_Rd_num, EXE_ALU_OP;
  wire [2:0] EXE_I_cmd;
  wire EXE_S, EXE_shift_imm, EXE_load_instr, EXE_PR_enable, EXE_RW;
  wire [1:0]EXE_DataSize;
  // Shifter/Sign Extender
  wire [31:0]EXE_SSE_Out;
  wire EXE_LS;
  // Pre-ALU Mux
  wire [31:0] EXE_ALU_MUX_Out;
  // ALU component
  wire [31:0] EXE_ALU_Out;
  wire ALU_N, ALU_Z, ALU_C, ALU_V;
  // ALUvsSSE Mux
  wire [31:0] EXE_ALUvsSSE_MUX_Out;
  // Status Register
  wire SR_N, SR_Z, SR_C, SR_V;
  // Condition Handler
  wire EXE_CH_B_Out, EXE_CH_I_EN;
  
  	// MEM wires
  // EXE/MEM Pipeline
  wire [31:0] MEM_DataMemIn, MEM_AddressDataOut;
  wire [3:0] MEM_Rd_Out;
  wire [1:0] MEM_DataSizeOut;
  wire MEM_LoadInst, MEM_RF_EN, MEM_RW;
  //DATA MEM
  wire [31:0]MEM_DataOut, MEM_AddressOut, MEM_MuxOut;
  
  
  	// WB wires
  //MEM/WB Pipeline
  wire [31:0]WB_LoadDataOut, WB_NonLoadDataOut;
  wire [3:0]WB_RdOut;
  wire WB_LoadInst, WB_RF_EN;
  //WB Mux
  wire [31:0] WB_DataOut;
  
    // MISC wires
  // Hazards Forwarding Unit
  wire [1:0] mux_S_A, mux_S_B, mux_S_D;
  wire Nop_insertion_S, IFID_enable, PC_enable;
  
  
  ////// MODULES //////
  
    // IF modules
  inst_fetch IF (IF_DataOut, IF_NextPC, IF_PC_In, ID_TargetAdress, RegPC_Out, EXE_CH_B_Out, TR, clk);
  	// ID modules
  IFID ifid_pipe (ID_PC_out, ID_instr_out, IF_NextPC, IF_DataOut, clk, EXE_CH_B_Out, IFID_enable, TR);
  RegisterFile regFile (PA, PB, RegPC_Out, PD, WB_DataOut, IF_PC_In, WB_RdOut, ID_instr_out[19:16], ID_instr_out[3:0], ID_instr_out[15:12], WB_RF_EN, PC_enable, clk, TR);
  mux_4x1_32Bit mux_A (ID_mux_A_out, mux_S_A, PA, EXE_ALUvsSSE_MUX_Out, MEM_MuxOut, WB_DataOut);
  mux_4x1_32Bit mux_B (ID_mux_B_out, mux_S_B, PB, EXE_ALUvsSSE_MUX_Out, MEM_MuxOut, WB_DataOut);
  mux_4x1_32Bit mux_D (ID_mux_D_out, mux_S_D, PD, EXE_ALUvsSSE_MUX_Out, MEM_MuxOut, WB_DataOut);
  x4_Sign_Extender x4_sign_ext (extended, ID_instr_out[23:0]);
  Adder adder (ID_TargetAdress, extended, ID_PC_out);
    
  control_unit CU (ID_ALU_OP_CU, ID_data_size_CU, ID_shift_imm_CU, ID_load_instr_CU, ID_RF_enable_CU, ID_B_instr, ID_RW_CU, ID_instr_out);
  CU_Mux CU_mux (ID_ALU_OP, ID_shift_imm, ID_load_instr, ID_RF_enable, ID_RW, ID_data_size, ID_ALU_OP_CU, ID_shift_imm_CU, ID_load_instr_CU, ID_RF_enable_CU, Nop_insertion_S, ID_RW_CU, ID_data_size_CU);
  
  	// EXE modules
  ID_EXE_pipeline idexe_pipe (EXE_In, EXE_A, EXE_B, EXE_immed, EXE_Opcode, EXE_Rd_num, EXE_ALU_OP, EXE_I_cmd, EXE_S, EXE_shift_imm, EXE_load_instr, EXE_RF_enable, EXE_RW, EXE_DataSize, ID_mux_A_out, ID_mux_B_out, ID_mux_D_out, ID_instr_out, ID_ALU_OP, ID_shift_imm, ID_load_instr, ID_RF_enable,  clk, ID_data_size, ID_RW_CU, EXE_CH_I_EN, TR);
  shifter_sign_extender SSE (EXE_SSE_Out, EXE_LS, EXE_B, EXE_A, EXE_immed, EXE_Opcode, EXE_I_cmd);
  ALU_mux ALU_MUX (EXE_ALU_MUX_Out, EXE_B, EXE_SSE_Out, EXE_shift_imm);
  ALU_component ALU (EXE_ALU_Out, ALU_N, ALU_Z, ALU_C, ALU_V, EXE_A, EXE_ALU_MUX_Out, EXE_ALU_OP, SR_C);
  ALUvsSSE_mux ALUvsSSE (EXE_ALUvsSSE_MUX_Out, EXE_ALU_Out, EXE_SSE_Out, EXE_LS);
  status_register SR (SR_N, SR_Z, SR_C, SR_V, ALU_N, ALU_Z, ALU_C, ALU_V, EXE_S);
  condition_handler CH (EXE_CH_B_Out, EXE_CH_I_EN, ID_instr_out[31:28], ID_B_instr, SR_N, SR_Z, SR_C, SR_V);
  
    // MEM modules
  EXE_MEM_Pipeline exemem_pipe (MEM_DataMemIn, MEM_AddressDataOut, MEM_Rd_Out, MEM_DataSizeOut, MEM_LoadInst, MEM_RF_EN, MEM_LS, MEM_RW, EXE_ALUvsSSE_MUX_Out, EXE_In, EXE_Rd_num, ID_data_size, EXE_load_instr, EXE_RF_enable, EXE_LS, ALU_N, ALU_Z, ALU_C, ALU_V, clk, TR, EXE_RW);
  memory DataMemory (MEM_DataOut, MEM_AddressOut, MEM_MuxOut, MEM_DataMemIn, MEM_AddressDataOut, MEM_RW, MEM_DataSizeOut, MEM_LoadInst, MEM_LS);

  	// WB modules
  MEM_WB_Pipeline memwb_pipe (WB_LoadDataOut, WB_NonLoadDataOut, WB_RdOut, WB_LoadInst, WB_RF_EN, MEM_DataOut, MEM_AddressOut, MEM_Rd_Out, MEM_LoadInst, MEM_RF_EN, clk, TR);
  writeback WB (WB_DataOut, WB_NonLoadDataOut, WB_LoadDataOut, WB_LoadInst);  
  
  	// MISC modules
  HazForwUnit haz_forw (mux_S_A, mux_S_B, mux_S_D, Nop_insertion_S, IFID_enable, PC_enable, EXE_Rd_num, MEM_Rd_Out, WB_RdOut, ID_instr_out[19:16], ID_instr_out[3:0], ID_instr_out[15:12], EXE_RF_enable, MEM_RF_EN, WB_RF_EN, EXE_load_instr, ID_shift_imm);
  integer i, fi, j, code;
  reg [31:0]data;
  initial #500 $finish; // Especifica cuando termina simulación  
  initial begin 
    for(i = 0; i <= 1023; i=i+1) IF.ram1.Mem[i] = 8'b0;

    fi = $fopen("testcode_arm_ppu_3.txt", "r");
    j = 0;
	  while(!$feof(fi)) begin
      code = $fscanf(fi, "%b", data);
      //Preloading instruction in every address of instruction and data ram
      IF.ram1.Mem[j] = data;
    	DataMemory.dataRam.Mem[j] = data;
      //$display (DataMemory.dataRam.Mem[j]);
      j = j + 1;
    end
  $fclose(fi);
  end
  initial fork
    
  /*
    // Profs test
    //IF.ram1.Mem[0] = 32'b11100000100000100101000000000101;
    //IF.ram1.Mem[4] = 32'b11100010010100110011000000000001;
    //IF.ram1.Mem[8] = 32'b00011010111111111111111111111101;
    //IF.ram1.Mem[12] = 32'b11100101110000010101000000000011;
    //IF.ram1.Mem[16] = 32'b11011011100000000000000000000001;
    // NOPs
    //IF.ram1.Mem[20] = 32'b0;
    //IF.ram1.Mem[24] = 32'b0;
    //IF.ram1.Mem[28] = 32'b0;
    //IF.ram1.Mem[32] = 32'b0;
    
    //Yaz's Test
    //IF.ram1.Mem[0] = 8'b11100011; // MOV R7, #12
    //IF.ram1.Mem[1] = 8'b10100000;
    //IF.ram1.Mem[2] = 8'b01110000;
    //IF.ram1.Mem[3] = 8'b00001100;
    //NOPS
    //IF.ram1.Mem[4] = 8'b0;
    //IF.ram1.Mem[5] = 8'b0;
    //IF.ram1.Mem[6] = 8'b0;
    //IF.ram1.Mem[7] = 8'b0;
    //IF.ram1.Mem[8] = 8'b0;
    //IF.ram1.Mem[9] = 8'b0;
    //IF.ram1.Mem[10] = 8'b0;
    //IF.ram1.Mem[11] = 8'b0;
    // Subs
    //IF.ram1.Mem[12] = 8'b0; //SUBS R7,R7, #2
    //IF.ram1.Mem[13] = 8'b0;
    //IF.ram1.Mem[14] = 8'b0;
    //IF.ram1.Mem[15] = 8'b0;
    //IF.ram1.Mem[16] = 8'b0; //SUBS R7,R7, #2
    //IF.ram1.Mem[17] = 8'b0;
    //IF.ram1.Mem[18] = 8'b0;
    //IF.ram1.Mem[19] = 8'b0;
    //NOPS
    //IF.ram1.Mem[20] = 8'b11100010;
    //IF.ram1.Mem[21] = 8'b01010111;
    //IF.ram1.Mem[22] = 8'b01110000;
    //IF.ram1.Mem[23] = 8'b00000010;
  */
  
  // Danny's test
  /*
    IF.ram1.Mem[0]  = 32'b11100011101000000111000000001100; // MOV R7, #12
    IF.ram1.Mem[4]  = 32'b11100000100000100010000101100111; // ADD R2, R2, R7, ROR #0d2
    IF.ram1.Mem[8]  = 32'b11100011010100100000000100110000; // CMP R2, #0x130
    IF.ram1.Mem[12] = 32'b11011010111111111111111111111110; // BLE -2
    IF.ram1.Mem[16] = 32'b11100000011100101010000000000111; // RSBS R10, R2, R7
    IF.ram1.Mem[20] = 32'b10101010000000000000000000000010; // BGE +2
    IF.ram1.Mem[24] = 32'b0; // NOP
    IF.ram1.Mem[28] = 32'b0; // NOP
    IF.ram1.Mem[32] = 32'b11100111111000100101000000001010; // STRB R2, [R7, +R10]
  */
  join

  //Running clock
  initial begin
      TR = 1'b1;
      clk = 1'b0;
      #1 TR = 1'b0;
      track = 0;
    repeat (200) begin 
      #5 clk = ~clk;
    
    end
      
  end
  initial begin
  /* Things to display to see it all working:
    -IF things:
    --Instruction (IF_DataOut)
    --NormalNextPC (RegPC_Out)
    --BranchNextPC (ID_TargetAdress)
    --NextPC (IF_NextPC)
    -ID things:
    --ID_instruction (ID_instr_out)
    --CONTROL SIGNALS:
    ---B_instr (ID_B_instr)
    ---load_instr (ID_load_instr)
    ---RF_en (ID_RF_enable)
    ---shift_imm (ID_shift_imm)
    ---RW (ID_RW)
    ---ALU_op (ID_ALU_OP)
    ---data_size (ID_data_size)
    --PA (PA)
    --PB (PB)
    --Mux_A (ID_mux_A_out)
    -- Mux_B (ID_mux_B_out)
    -EXE things:
    --CONTROL SIGNALS
    --SSE Out (EXE_SSE_Out)
    --LS_cmd (EXE_LS)
    --A (EXE_A)
    --ALU mux (EXE_ALU_MUX_Out)
    --ALU out (EXE_ALU_Out)
    --ALU CCs (ALU_N, ALU_Z, ALU_C, ALU_V)
    --Status Register out (SR_N, SR_Z, SR_C, SR_V)
    --Condition Handler flag (EXE_CH_Out)
    --EXE mux (EXE_ALUvsSSE_MUX_Out)
    -MEM things:
    --CONTROL SIGNALS
    --Data Memory Out (MEM_DataOut)
    --Mem Mux (MEM_MuxOut)
    -WB things:
    --Target Register (WB_RdOut)
    --WB out (WB_DataOut)
    -Hazard Unit things:
    --S for ID_Mux_A (mux_S_A)
    --S for ID_Mux_B (mux_S_B)
    --Control Unit NOP insertion (Nop_insertion_S)
    --IFID pipeline enable (IFID_enable)
    --Register File PC enable (PC_enable)
  */

  
    // $display ("IF: IF_PC   Branch_PC      PC              Instruction             || ID:           Instruction           B LI RF SI RW ALU_OP DaSi            RF_A_Out                          RF_B_Out                         ID_A_Out                         ID_B_Out             || EXE: LI RF SI RW ALU_OP DaSi             SSE_Out              LS             ALU_A                              ALU_B                           ALU_Out             N Z C V  SRN SRZ SRC SRV CH_B CH_I_EN              EXE_Out              || MEM: LI RF RW DaSi           Data_Mem_Out                       Mux_Out              || WB: LI RF  Rd              Data_Out             || HU: ID_S_A ID_S_B CU_Nop IFID_EN PC_EN ||| Clock: |||         Time:");
    // $monitor ("%d %d %d %b  ||    %b %b %b  %b  %b  %b   %b   %b  %b %b %b %b ||      %b  %b  %b  %b  %b    %b  %b %b  %b %b %b %b %b %b %b   %b   %b   %b   %b    %b     %b    %b  ||      %b  %b  %b   %b  %b %b ||     %b  %b  %b %b ||       %b     %b     %b      %b      %b    |||    %b   ||| %d ", 
          	// RegPC_Out, ID_TargetAdress, IF_NextPC, IF_DataOut, 
          	// ID_instr_out, ID_B_instr, ID_load_instr, ID_RF_enable, ID_shift_imm, ID_RW, ID_ALU_OP, ID_data_size, PA, PB, ID_mux_A_out, ID_mux_B_out,
    			// EXE_load_instr, EXE_RF_enable, EXE_shift_imm, EXE_RW, EXE_ALU_OP, EXE_DataSize, EXE_SSE_Out, EXE_LS, EXE_A, EXE_ALU_MUX_Out, EXE_ALU_Out, 
    			  // ALU_N, ALU_Z, ALU_C, ALU_V, SR_N, SR_Z, SR_C, SR_V, EXE_CH_B_Out, EXE_CH_I_EN, EXE_ALUvsSSE_MUX_Out,
    			// MEM_LoadInst, MEM_RF_EN, MEM_RW, MEM_DataSizeOut, MEM_DataOut, MEM_MuxOut,
    			// WB_LoadInst, WB_RF_EN, WB_RdOut, WB_DataOut,
    			// mux_S_A, mux_S_B, Nop_insertion_S, IFID_enable, PC_enable, clk, $time);
    
    /*
      $display ("        Mem_Local      DataMemAtLocation                Time");
    	$monitor ("%d     %d   %d", track, DataMemory.dataRam.Mem[track], $time);
    */
    $display ("|||______PC_____||_MEM_AddressDataOut_||______R1_____|______R2_____|______R3_____|______R5_____||_Mem[43]_|||");
    $monitor ("||| %d || %d         || %d | %d | %d | %d ||   %d   |||", regFile.R15.Q, MEM_AddressDataOut, regFile.R1.Q, regFile.R2.Q, regFile.R3.Q, regFile.R5.Q, DataMemory.dataRam.Mem[43]);
    
  end
endmodule