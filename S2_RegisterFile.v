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

//Test for Register File (WIP)
module test_RegisterFile;
    //Inputs
    reg [31:0] PW, PC_in;
    reg [3:0] RW, RA, RB;
    reg LE, PCLd, Clk;

    //Outputs
    wire [31:0] PuertoA, PuertoB, PC_out;

    //Instantiating RegisterFile module
    RegisterFile regFile (PuertoA, PuertoB, PC_out, PW, PC_in, RW, RA, RB, LE, PCLd, Clk);

    //Running clock
    initial begin
        Clk = 1'b0;
        repeat (40)
        #5 Clk = ~Clk;
    end

        initial begin
        RA = 0;
        RB = 1;
        RW = 0;
        repeat (15) #10 RW = RW + 1;
        #10 LE = 0;//Finished writing
        fork 
        repeat (8) #5 RA = RA + 2;
        repeat (8) #5 RB = RB + 2;
        join
        end   

    initial fork
        PC_in = 0;
        PCLd = 0;
        LE = 1;
        PW = 0;
        repeat (15) #10 PW = PW + 1;
    join

    initial begin
        $display ("  PW        PuertoA    PuertoB    PC_out  |||| RW    RA    RB   LE  Clk");
        $monitor ("%h | %h | %h | %h |||| %b  %b  %b  %b   %b ", PW, PuertoA, PuertoB, PC_out, RW, RA, RB, LE, Clk);
    end

endmodule

//-----------------Módulos para ser instanciados en este file------------------------------//
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

module mux_2x1_32Bit (output reg [31:0] Y, input S, input [31:0] A, B);
    always @ (S, A, B)
        if (!S) Y = B;
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

module register_32bit (output reg [31:0] Q, input [31:0] D, input Clk, Ld);
    always @ (posedge Clk)
        if(Ld) Q <= D;
endmodule