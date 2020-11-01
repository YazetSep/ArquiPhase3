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

module ALU_mux(output reg [31:0] Out, input [31:0] B, immed, input shift_imm);
  always @ (shift_imm, B, immed)
      if (shift_imm) Out = immed;
      else Out = B;
endmodule

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

// Decides which address to pass onto the MEM stage
module ALUvsSSE_mux(output reg [31:0] Out, input [31:0] ALU_Out, SSE_Out, input LS);
  always @ (LS, ALU_Out, SSE_Out)
      if (LS) Out = SSE_Out;
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

module status_register(output reg N, Z, C, V, input Ni, Zi, Ci, Vi, S);
  always @ (Ni, Zi, Ci, Vi, S)
    begin 
      N = 1'b0; Z = 1'b0; C = 1'b0; V = 1'b0;
      if (S) begin
        N = Ni; Z = Zi; C = Ci; V = Vi;
      end
    end
endmodule

/* TESTING */

module ALU_test; 
  reg [31:0] A, B;
  reg [3:0] ALU_op;
  reg S, Ci;
  wire [31:0] Out;
  wire N, Z, C, V;
  ALU_component AU (Out, N, Z, C, V, A, B, ALU_op, Ci); //instancia ALU
  initial #100 $finish; // Especifica cuando termina simulación  
  initial fork
    #1 A = 32'b0000_0000_0000_0000_0000_0000_0000_0101;
    #1 B = 32'b0000_0000_0000_0000_0000_0111_0001_1101;
    #1  ALU_op <= 4'b0000; #1  S <= 1'b0; #1  Ci <= 1'b1; // AND: Logical AND
    #2  ALU_op <= 4'b0001; #2  S <= 1'b0; #2  Ci <= 1'b1; // EOR: Logical Exclusive OR
    #3  ALU_op <= 4'b0010; #3  S <= 1'b0; #3  Ci <= 1'b1; // SUB: Subtract
    #4  ALU_op <= 4'b0011; #4  S <= 1'b0; #4  Ci <= 1'b1; // RSB: Reverse Subtract
    #5  ALU_op <= 4'b0100; #5  S <= 1'b0; #5  Ci <= 1'b1; // ADD: Add
    #6  ALU_op <= 4'b0101; #6  S <= 1'b0; #6  Ci <= 1'b1; // ADC: Add with Carry
    #7  ALU_op <= 4'b0110; #7  S <= 1'b0; #7  Ci <= 1'b1; // SBC: Subtract with Carry
    #8  ALU_op <= 4'b0111; #8  S <= 1'b0; #8  Ci <= 1'b1; // RSC: Reverse Subtract with Carry
    #9  ALU_op <= 4'b1000; #9  S <= 1'b0; #9  Ci <= 1'b1; // TST: Test
    #10 ALU_op <= 4'b1001; #10 S <= 1'b0; #10 Ci <= 1'b1; // TEQ: Test Equivalence
    #11 ALU_op <= 4'b1010; #11 S <= 1'b0; #11 Ci <= 1'b1; // CMP: Compare
    #12 ALU_op <= 4'b1011; #12 S <= 1'b0; #12 Ci <= 1'b1; // CMN: Compare Negated
    #13 ALU_op <= 4'b1100; #13 S <= 1'b0; #13 Ci <= 1'b1; // ORR: Logical Or
    #14 ALU_op <= 4'b1101; #14 S <= 1'b0; #14 Ci <= 1'b1; // MOV: Move
    #15 ALU_op <= 4'b1110; #15 S <= 1'b0; #15 Ci <= 1'b1; // BIC: Bit Clear
    #16 ALU_op <= 4'b1111; #16 S <= 1'b0; #16 Ci <= 1'b1; // MVN: Move Not
    // ADD and SUB cases
    // Overflows
    #17 A = 32'b0100_0000_0000_0000_0000_0000_0000_0101;
    #17 B = 32'b0100_0000_0000_0000_0000_0111_0001_1101;
    #17 ALU_op = 4'b0100; #17 S = 1'b1; #17 Ci = 1'b1; // ADD: Add
    #18 A = 32'b0100_0000_0000_0000_0000_0000_0000_0101;
    #18 B = 32'b1000_0000_0000_0000_0000_0111_0001_1101;
    #18 ALU_op = 4'b0010; #18 S = 1'b1; #18 Ci = 1'b1; // SUB: Subtract

    // Doesn't Overflow
    #19 A = 32'b0110_0000_0000_0000_0000_0000_0000_0101;
    #19 B = 32'b0100_0000_0000_0000_0000_0111_0001_1101;
    #19 ALU_op = 4'b0110; #19 S = 1'b1; #19 Ci = 1'b1; // SBC: Subtract with Carry
    #20 A = 32'b0000_0000_0000_0000_0000_0000_0000_0101;
    #20 B = 32'b0100_0000_0000_0000_0000_0111_0001_1101;
    #20 ALU_op = 4'b0101; #20 S = 1'b1; #20 Ci = 1'b1; // ADC: Add with Carry
  join
  initial begin
    $display ("ALU: Oper              A(b)                    A(d)                 B(b)                    B(d)                Out(b)                 Out(d)  Ci S N Z C V    Time:");
    #1 $monitor ("     %b %b %d %b %d  %b %d %b  %b %b %b %b %b %d", ALU_op, A, A,
    B, B, Out, Out, Ci, S, N, Z, C, V, $time);
  end
endmodule

module test_shifter;
  reg [31:0] Rm, Rn; reg [11:0] I; reg[4:0] Opcode; reg [2:0] I_cmd;
  wire [31:0] Out; wire LS;
  shifter_sign_extender sse (Out, LS, Rm, Rn, I, Opcode, I_cmd);
  initial #100 $finish; // Especifica cuando termina simulación
  initial fork
    #31 Rm = 32'b1110_1011_0000_0000_0000_0000_0000_0111;
    #31 Rn = 32'b0000_1000_0000_0000_0000_0011_0000_1001;
    #31 Opcode = 5'b10101;
    #31 I_cmd = 3'b001; #31 I = 12'b0000_01101010; // Immediate
    #32 I_cmd = 3'b001; #32 I = 12'b0010_0110_1010; // Shift by Immediate
    #33 I_cmd = 3'b000; #33 I = 12'b0001_0000_0101; // LSL: Logical Shift Left 
    #34 I_cmd = 3'b000; #34 I = 12'b0001_1010_0101; // LSR: Logical Shift Right
    #35 I_cmd = 3'b000; #35 I = 12'b0010_1100_0101; // ASR: Arithmetic Shift Right
    #36 I_cmd = 3'b000; #36 I = 12'b0010_1110_0101; // ROR: Rotate Right
    #37 I_cmd = 3'b010; #37 I = 12'b0010_1110_0101; // Load/Store Immediate offset (Subtracts)
    #38 I_cmd = 3'b011; #38 Opcode = 5'b11101; // Load/Store Register offset (Adds)
  join
  initial begin
    #21 $display ("SSE:                Rm                               Rn                     I       I_cmd                Out                LS     Time:");
    #10 $monitor ("     %b %b %b  %b  %b  %b %d ", Rm, Rn, I, I_cmd, Out, LS, $time); 
  end
endmodule

module test_condition_handler;
  reg [3:0] cond; reg B_instr, N, Z, C, V;
  wire out;
  condition_handler ch (out, cond, B_instr, N, Z, C, V);
  initial #100 $finish; // Especifica cuando termina simulación
  initial fork
    #40 B_instr = 1'b0; #40 cond = 4'b0000;
    #40 N = 1'b1; Z = 1'b1; #40 C = 1'b1; #40 V = 1'b1; // return: 0 - (B_instr is off)
    #41 cond = 4'b0000; #41 B_instr = 1'b1;   // return: 1 - Equal
    #42 cond = 4'b0001;                      // return: 0 - Not equal
    #43 cond = 4'b0010;                     // return: 1 - Unsigned	higher or same
    #44 cond = 4'b0011;                     // return: 0 - Unsigned lower
    #45 cond = 4'b0100;                     // return: 1 - Minus
    #46 cond = 4'b0101;                     // return: 0 - Positive or Zero
    #47 cond = 4'b0110;                     // return: 1 - Overflow
    #48 cond = 4'b0111;                     // return: 0 - No overflow
    #49 cond = 4'b1000; #49 Z = 1'b0;        // return: 1 - Unsigned higher
    #50 cond = 4'b1001;                    // return: 0 - Unsigned lower or same
    #51 cond = 4'b1010;                     // return: 1 - Greater or equal
    #52 cond = 4'b1011; #52 V = 1'b0;       // return: 1 - Less than
    #53 cond = 4'b1100;                     // return: 0 - Greater than
    #54 cond = 4'b1101;                     // return: 0 - Less than or equal
    // Always: return: 1
    #55 cond = 4'b1110; #55 N = 1'b0; #55 Z = 1'b0; #55 C = 1'b0; #55 V = 1'b0;
  join
  initial begin
    #39 $display ("CH: B cond N Z C V out            Time:");
    #1 $monitor ("    %b %b %b %b %b %b  %b  %d ", B_instr, cond, N, Z, C, V, out, $time); 
  end
endmodule

module EXE_test;
  // External inputs
  reg [31:0] A, B;
  reg [11:0] I;
  reg [4:0] Opcode;
  reg [3:0] ALU_op, cond;
  reg [2:0] I_cmd;
  reg shift_imm, B_instr, S;
  // Internal Outputs
  wire [31:0] ALU_MUX_Out, ALU_Out, SSE_Out, F_Out;
  wire LS, ALU_N, ALU_Z, ALU_C, ALU_V, Ni, Zi, Ci, Vi, CH_Out; 

  ALU_component AU (ALU_Out, ALU_N, ALU_Z, ALU_C, ALU_V, A, ALU_MUX_Out, ALU_op, Ci); 
  shifter_sign_extender SSE (SSE_Out, LS, B, A, I, Opcode, I_cmd);
  condition_handler CH (CH_Out, cond, B_instr, Ni, Zi, Ci, Vi);
  ALU_mux AM (ALU_MUX_Out, B, SSE_Out, shift_imm);
  ALUvsSSE_mux ASM(F_Out, ALU_Out, SSE_Out, LS);
  status_register SR (Ni, Zi, Ci, Vi, ALU_N, ALU_Z, ALU_C, ALU_V, S);

  initial #100 $finish;
  initial #57 fork

    #1 begin
      A = 32'b0000_0000_0000_0000_0000_0000_0000_0101;
      B = 32'b0000_0000_0000_0000_0000_0111_0001_1101;
      I = 12'b0000_01101010;
      Opcode = 5'b10101;
      ALU_op = 4'b0000; 
      cond = 4'b0000;
      I_cmd = 3'b001; 
      shift_imm = 1'b0;
      S = 1'b0;
      B_instr = 1'b0;
    end
    #2 B_instr = 1'b1;
    #3 shift_imm = 1'b1;
    #4 S = 1'b1;
    #5 begin
      A = 32'b0100_0000_0000_0000_0000_0000_0000_0101;
      B = 32'b0100_0000_0000_0000_0000_0111_0001_1101; 
      I_cmd = 3'b011;
    end
  join
  initial begin
    #57 $display ("EXE:              A(Rn)                             B(Rm)                    I      I_cmd Opcode            SSE_Out               SI           ALU_MUX_Out            ALU_op Ci             ALU_Out              N Z C V LS               F_Out              S Nf Zf Cf Vf BI cond CH_Out          Time:");
    #1 $monitor ("     %b %b %b  %b  %b  %b %b  %b  %b  %b  %b %b %b %b %b %b  %b %b %b  %b  %b  %b  %b  %b   %b    %d", A, B, I, I_cmd, Opcode, SSE_Out, shift_imm, 
    ALU_MUX_Out, ALU_op, Ci, ALU_Out, ALU_N, ALU_Z, ALU_C, ALU_V, LS, F_Out, S, Ni, Zi, Ci, Vi, B_instr, cond, CH_Out, $time);
  end
endmodule


// module test_SR;
//   reg S, N, Z, C, V;
//   wire Nf, Zf, Cf, Vf;
//   status_register SR(Nf, Zf, Cf, Vf, N, Z, C, V, S);
//   initial #100 $finish; // Especifica cuando termina simulación
//   initial fork
//     N = 1'b0; Z = 1'b1; C = 1'b1; V = 1'b0;
//     S = 1'b0;
//     #1 S = 1'b1;
//   join
//   initial begin
//     $display ("SR: N Z C V S  Nf Zf Cf Vf           Time:");
//     $monitor ("    %b %b %b %b %b  %b  %b  %b  %b  %d ", N, Z, C, V, S, Nf, Zf, Cf, Vf,  $time); 
//   end
// endmodule