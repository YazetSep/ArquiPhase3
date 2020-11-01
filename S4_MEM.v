module memory(output reg [31:0]DataOut, AddressOut, MuxOut, input [31:0]DataIn, AddressIn, input RW, input [1:0]DataSize, input MuxController, Enable);
always @(*)
    begin
        ram256x32_data dataRam (DataOut, Enable, RW, DataIn, AddressIn, DataSize);
        mux_2x1_32Bit mux (MuxOut, MuxController, DataOut, AddressOut);
        AddressOut = AddressIn;
    end
endmodule

module mux_2x1_32Bit (output reg [31:0] Y, input S, input [31:0] A, B);
    always @ (*)
        if (!S) Y = B;
        else Y = A;
endmodule