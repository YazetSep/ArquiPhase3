module writeback (output reg [31:0]DataOut, input [31:0]AddressIn, input[31:0]DataIn, input LoadInst)
always @(*)
begin
    mux_2x1_32Bit mux (DataOut, LoadInst, DataIn, AddressIn);
end


endmodule

module mux_2x1_32Bit (output reg [31:0] Y, input S, input [31:0] A, B);
    always @ (*)
        if (!S) Y = B;
        else Y = A;
endmodule