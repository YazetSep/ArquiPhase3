module inst_fetch(output reg [31:0]DataOut, output reg [31:0]NextPC, output reg [31:0]PC_In, input [31:0]TargetAddress, input [31:0]PC, input condition_handler_in);
    wire [31:0]DOut;
    wire [31:0]MOut;
    
    ram256x32_inst ram1 (DOut, 1'b1, PC);
    
    mux_2x1_32Bit mux (MOut, condition_handler_in, NextPC, TargetAddress);
    
    always @(*)
        begin
            NextPC <= PC + 4;
            #5 DataOut <= DOut;
            #5 PC_In <= MOut;
        end
endmodule

module mux_2x1_32Bit (output reg [31:0] Y, input S, input [31:0] A, B);
    always @ (*)
        if (!S) Y = B;
        else Y = A;
endmodule