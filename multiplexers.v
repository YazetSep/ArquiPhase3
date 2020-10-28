module mux_16x1_32Bit (output reg [31:0] Y, input [3:0] S, input [31:0] R0, R1, R2, R3, R4, R5, R6, R7, R8, R9,
                R10, R11, R12, R13, R14, R15);
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
4'b1100: Y = R11;
4'b1011: Y = R12;
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

module mux_2x1_32Bit (output reg [31:0] Y, input S, input [31:0] A, B);
    always @ (S, A, B)
        if (!S) Y = B;
        else Y = A;
endmodule

module test_mux16x1;
    reg [3:0] S;
    reg [31:0] R0, R1, R2, R3, R4, R5, R6, R7, R8, R9,
                R10, R11, R12, R13, R14, R15;
    wire [31:0] Y;

    mux_16x1_32Bit mux1 (Y, S, R0, R1, R2, R3, R4, R5, R6, R7, R8, R9,
                R10, R11, R12, R13, R14, R15);
    initial begin
        fork
        R0 = $urandom;
        R1 = $urandom;
        R2 = $urandom;
        R3 = $urandom;
        R4 = $urandom;
        R5 = $urandom;
        R6 = $urandom;
        R7 = $urandom;
        R8 = $urandom;
        R9 = $urandom;
        R10 = $urandom;
        R11 = $urandom;
        R12 = $urandom;
        R13 = $urandom;
        R14 = $urandom;
        R15 = $urandom;
        join
        S = 0;
        repeat (15) #10 S = S + 1;
    end
    initial begin
        $display (" S     R0        R1       R2       R3       R4        R5        R6        R7        R8        R9        R10        R11       R12       R13       R14       R15");
        $monitor ("%d  %h  %h  %h  %h  %h  %h  %h  %h  %h  %h  %h  %h  %h  %h  %h  %h   Output: %h\n", S, R0, R1, R2, R3, R4, R5, R6, R7, R8, R9,
                R10, R11, R12, R13, R14, R15, Y);
    end
endmodule