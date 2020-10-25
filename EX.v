module ALU_component (output reg [31:0] Out, output reg N, Z, C, V, // CC = N, Z, C, V
      input [31:0] A, B, input [3:0] ALU_op, input Ci); 
  reg temp;
  reg [31:0]tempOut; // For cases where output doesn't matter
  always @ (ALU_op,A,B)
  begin
    N = 1'b0; Z = 1'b0; C = 1'b0; V = 1'b0;
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
    if ((ALU_op >= 4'b0010) & (ALU_op <= 4'b0111)) V = (Out[31] && ~(A[31] & B[31])) ^ (A[31] | B[31]);
    
    if ((ALU_op >= 4'b1000) & (ALU_op <= 4'b1011)) begin
      N = tempOut[31];
      Z = tempOut && 32'b0;
      V = tempOut[31] && ~(A[31] & B[31]);
      Out = 32'b0;
      end
  end
    
endmodule

module ALU_mux (output reg [31:0] Out, input [31:0] B, immed, input shift_imm);
  always @ (shift_imm, B, immed)
      if (shift_imm) Out = shift_imm;
      else Out = B;
endmodule

module shifter_sign_extender(output reg [31:0] Out, input [31:0] Rm, input[31:0] Rn, 
      input [11:0] I, input[4:0] Opcode, input [2:0] I_cmd);
  reg temp1;
  always @ (Rm, Rn, I, Opcode, I_cmd)
  begin
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
      3'b010 : if (Opcode[3]) Out = Rn + I; else Out = Rn - I;  // Load/Store Immediate offset
      3'b011 : if (Opcode[3]) Out = Rn + Rm; else Out = Rn - Rm;  // Load/Store Register offset
      default : Out = Rm;
    endcase
  end
endmodule

// Decides which address to pass onto the MEM stage
module ALUvsSSE_mux (output reg [31:0] Out, input [31:0] ALU_Out, SSE_Out, input load_instr);
  always @ (load_instr, ALU_Out, SSE_Out)
      if (load_instr) Out = SSE_Out;
      else Out = ALU_Out;
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

/* TESTING */

module ALU_test; 
  reg [31:0] A, B;
  reg [3:0] ALU_op;
  reg S, Ci;
  wire [31:0] Out;
  wire Nf, Zf, Cf, Vf;
  ALU_component AU (Out, Nf, Zf, Cf, Vf, A, B, ALU_op, Ci); //instancia ALU
  initial #70 $finish; // Especifica cuando termina simulación  
  initial fork
    #2 A = 32'b0000_0000_0000_0000_0000_0000_0000_0101;
    #2 B = 32'b0000_0000_0000_0000_0000_0111_0001_1101;
    #2  ALU_op <= 4'b0000; #2  S <= 1'b0; #2  Ci <= 1'b1; // AND: Logical AND
    #4  ALU_op <= 4'b0001; #4  S <= 1'b0; #4  Ci <= 1'b1; // EOR: Logical Exclusive OR
    #6  ALU_op <= 4'b0010; #6  S <= 1'b0; #6  Ci <= 1'b1; // SUB: Subtract
    #8  ALU_op <= 4'b0011; #8  S <= 1'b0; #8  Ci <= 1'b1; // RSB: Reverse Subtract
    #10 ALU_op <= 4'b0100; #10 S <= 1'b0; #10 Ci <= 1'b1; // ADD: Add
    #12 ALU_op <= 4'b0101; #12 S <= 1'b0; #12 Ci <= 1'b1; // ADC: Add with Carry
    #14 ALU_op <= 4'b0110; #14 S <= 1'b0; #14 Ci <= 1'b1; // SBC: Subtract with Carry
    #16 ALU_op <= 4'b0111; #16 S <= 1'b0; #16 Ci <= 1'b1; // RSC: Reverse Subtract with Carry
    #18 ALU_op <= 4'b1000; #18 S <= 1'b0; #18 Ci <= 1'b1; // TST: Test
    #20 ALU_op <= 4'b1001; #20 S <= 1'b0; #20 Ci <= 1'b1; // TEQ: Test Equivalence
    #22 ALU_op <= 4'b1010; #22 S <= 1'b0; #22 Ci <= 1'b1; // CMP: Compare
    #24 ALU_op <= 4'b1011; #24 S <= 1'b0; #24 Ci <= 1'b1; // CMN: Compare Negated
    #26 ALU_op <= 4'b1100; #26 S <= 1'b0; #26 Ci <= 1'b1; // ORR: Logical Or
    #28 ALU_op <= 4'b1101; #28 S <= 1'b0; #28 Ci <= 1'b1; // MOV: Move
    #30 ALU_op <= 4'b1110; #30 S <= 1'b0; #30 Ci <= 1'b1; // BIC: Bit Clear
    #32 ALU_op <= 4'b1111; #32 S <= 1'b0; #32 Ci <= 1'b1; // MVN: Move Not
    // ADD and SUB cases
    // Overflows
    #34 A = 32'b0100_0000_0000_0000_0000_0000_0000_0101;
    #34 B = 32'b0100_0000_0000_0000_0000_0111_0001_1101;
    #34 ALU_op = 4'b0100; #34 S = 1'b1; #34 Ci = 1'b1; // ADD: Add
    #36 A = 32'b1100_0000_0000_0000_0000_0000_0000_0101;
    #36 B = 32'b1110_0000_0000_0000_0000_0111_0001_1101;
    #36 ALU_op = 4'b0010; #36 S = 1'b1; #36 Ci = 1'b1; // SUB: Subtract

    // Doesn't Overflow
    #38 A = 32'b0110_0000_0000_0000_0000_0000_0000_0101;
    #38 B = 32'b0100_0000_0000_0000_0000_0111_0001_1101;
    #38 ALU_op = 4'b0110; #38 S = 1'b1; #38 Ci = 1'b1; // SBC: Subtract with Carry
    #40 A = 32'b0000_0000_0000_0000_0000_0000_0000_0101;
    #40 B = 32'b0100_0000_0000_0000_0000_0111_0001_1101;
    #40 ALU_op = 4'b0101; #40 S = 1'b1; #40 Ci = 1'b1; // ADC: Add with Carry
  join
  initial begin
    $display ("Oper              A(b)                    A(d)                 B(b)                    B(d)                Out(b)                 Out(d)  Ci S N Z C V    Time:");
    $monitor ("%b %b %d %b %d  %b %d %b  %b %b %b %b %b %d", ALU_op, A, A,
    B, B, Out, Out, Ci, S, Nf, Zf, Cf, Vf, $time);
  end
endmodule

module test_shifter;
  reg [31:0] Rm, Rn; reg [11:0] I; reg[4:0] Opcode; reg [2:0] I_cmd;
  wire [31:0] Out;
  shifter_sign_extender sse (Out, Rm, Rn, I, Opcode, I_cmd);
  initial #50 $finish; // Especifica cuando termina simulación
  initial fork
    Rm = 32'b1110_1011_0000_0000_0000_0000_0000_0111;
    Rn = 32'b0000_1000_0000_0000_0000_0011_0000_1001;
    Opcode = 5'b10101;
    I_cmd = 001; I = 12'b0000_01101010; // Immediate
    #2 I_cmd = 3'b001;  #2  I = 12'b0010_0110_1010; // Shift by Immediate
    #4 I_cmd = 3'b000;  #4  I = 12'b0001_0000_0101; // LSL: Logical Shift Left 
    #6 I_cmd = 3'b000;  #6  I = 12'b0001_1010_0101; // LSR: Logical Shift Right
    #8 I_cmd = 3'b000;  #8  I = 12'b0010_1100_0101; // ASR: Arithmetic Shift Right
    #10 I_cmd = 3'b000; #10 I = 12'b0010_1110_0101; // ROR: Rotate Right
    #12 I_cmd = 3'b010; #12 I = 12'b0010_1110_0101; // Load/Store Immediate offset (Subtracts)
    #14 I_cmd = 3'b011; #14 Opcode = 5'b11101; // Load/Store Register offset (Adds)
  join
  initial begin
    $display ("                Rm                               Rn                     I       I_cmd                Out                    Time:");
    $monitor (" %b %b %b  %b  %b %d ", Rm, Rn, I, I_cmd, Out, $time); 
  end
endmodule

module test_condition_handler;
  reg [3:0] cond; reg B_instr, N, Z, C, V;
  wire out;
  condition_handler ch (out, cond, B_instr, N, Z, C, V);
  initial #50 $finish; // Especifica cuando termina simulación
  initial fork
    B_instr = 1'b0; cond = 4'b0000;
    N = 1'b1; Z = 1'b1; C = 1'b1; V = 1'b1; // return: 0 - (B_instr is off)
    #1 cond = 4'b0000; #1 B_instr = 1'b1;   // return: 1 - Equal
    #2 cond = 4'b0001;                      // return: 0 - Not equal
    #3  cond = 4'b0010;                     // return: 1 - Unsigned	higher or same
    #4  cond = 4'b0011;                     // return: 0 - Unsigned lower
    #5  cond = 4'b0100;                     // return: 1 - Minus
    #6  cond = 4'b0101;                     // return: 0 - Positive or Zero
    #7  cond = 4'b0110;                     // return: 1 - Overflow
    #8  cond = 4'b0111;                     // return: 0 - No overflow
    #9  cond = 4'b1000; #9 Z = 1'b0;        // return: 1 - Unsigned higher
    #10  cond = 4'b1001;                    // return: 0 - Unsigned lower or same
    #11 cond = 4'b1010;                     // return: 1 - Greater or equal
    #12 cond = 4'b1011; #12 V = 1'b0;       // return: 1 - Less than
    #13 cond = 4'b1100;                     // return: 0 - Greater than
    #14 cond = 4'b1101;                     // return: 0 - Less than or equal
    // Always: return: 1
    #15 cond = 4'b1110; #15 N = 1'b0; #15 Z = 1'b0; #15 C = 1'b0; #15 V = 1'b0;
  join
  initial begin
    $display (" B cond N Z C V out            Time:");
    $monitor (" %b %b %b %b %b %b  %b  %d ", B_instr, cond, N, Z, C, V, out, $time); 
  end
endmodule