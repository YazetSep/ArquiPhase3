module EXE_MEM_Pipeline(output reg [31:0]DataMemIn, output reg [31:0]AddressDataOut, output reg [3:0]Rd_Out, output reg [1:0]DataSizeOut, output reg LoadInst, output reg RF_EN, input [1:0]DataSizeIn, input [31:0]ALU_Out, input N, Z, C, V, input [31:0]DataIn, input [3:0]Rd_In, input LI, input RF_Enable)
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