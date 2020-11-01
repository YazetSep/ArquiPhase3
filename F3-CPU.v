//////////// Instrcution Fetch (IF) ///////////

/* IF/ID */
module IFID (output reg [31:0] PC_out, instr_out, input [31:0] PC_in, instr_in, input clk, reset, LE);
always @(posedge clk, posedge reset) begin
    if (reset)
        instr_out <= 0; //Control hazard handling reset
    else if (LE) begin //LE is used to stall when load hazard is asserted from the Haz/Forw Unit
        PC_out <= PC_in;
        instr_out <= instr_in;
    end
end
endmodule

//////////// Instruction Decode (ID) //////////

module x4_Sign_Extender(output reg [31:0] extended, input [23:0] input_instr);
    always @ (*) begin
        extended = {{8{1'b0}}, input_instr};
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

module RegisterFile (output [31:0] PuertoA, PuertoB, PC_out, input [31:0] PW, PC_in, input [3:0] RW, RA, RB, input LE, PCLd, Clk);
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
    //Multiplexer to control PC_in (Priority writing data instead of writing new PC value)
    mux_2x1_32Bit mux_2x1PC_in (mux_PCOut, E[15], PW, PC_in);
    //OR gate to activate load of Register15/PC with Binary Decoder E[15] port or PCLd port
    or (reg15Ld, E[15], PCLd);
    //Registers 0 to 15
    register_32bit R0 (Q0, PW, Clk, E[0]);
    register_32bit R1 (Q1, PW, Clk, E[1]);
    register_32bit R2 (Q2, PW, Clk, E[2]);
    register_32bit R3 (Q3, PW, Clk, E[3]);
    register_32bit R4 (Q4, PW, Clk, E[4]);
    register_32bit R5 (Q5, PW, Clk, E[5]);
    register_32bit R6 (Q6, PW, Clk, E[6]);
    register_32bit R7 (Q7, PW, Clk, E[7]);
    register_32bit R8 (Q8, PW, Clk, E[8]);
    register_32bit R9 (Q9, PW, Clk, E[9]);
    register_32bit R10 (Q10, PW, Clk, E[10]);
    register_32bit R11 (Q11, PW, Clk, E[11]);
    register_32bit R12 (Q12, PW, Clk, E[12]);
    register_32bit R13 (Q13, PW, Clk, E[13]);
    register_32bit R14 (Q14, PW, Clk, E[14]);
    register_32bit R15 (Q15, mux_PCOut, Clk, reg15Ld);//Special Register also functions as Program Counter (PC)

    assign PC_out = Q15;

endmodule

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

module EXE_MEM_Pipeline(output reg [31:0] DataMemIn, AddressDataOut, output reg [3:0]Rd_Out, output reg [1:0]DataSizeOut, output reg LoadInst, RF_EN, 
input [31:0]ALU_Out, DataIn, input [3:0] Rd_In, input [1:0]DataSizeIn, input LI, RF_Enable, N, Z, C, V, clk);
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

module HazForwUnit (output reg [1:0] mux_S_A, mux_S_B, output reg Nop_insertion_S, IFID_enable, PC_enable,
input [3:0] EX_Rd, MEM_Rd, WB_Rd, ID_Rn, ID_Rm, 
input EX_RF_enable, MEM_RF_enable, WB_RF_enable, EX_load_instr);
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
    
        //Detecting and Handling Load Hazard
        if (EX_load_instr && ((ID_Rn === EX_Rd) || (ID_Rm === EX_Rd))) begin
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
endmodule