module inst_fetch(output reg [31:0]DataOut, output reg [31:0]NextPC, output reg [31:0]PC_In, input [31:0]TargetAddress, input [31:0]PC, input condition_handler_in)
    always@(*)
        begin
            ram256x32_inst ram1 (DataOut, 1'b1, PC);
            NextPC = PC + 4;
            mux_2x1_32Bit mux (PC_In, condition_handler_in, NextPC, TargetAddress);   
        end
endmodule

module mux_2x1_32Bit (output reg [31:0] Y, input S, input [31:0] A, B);
    always @ (*)
        if (!S) Y = B;
        else Y = A;
endmodule