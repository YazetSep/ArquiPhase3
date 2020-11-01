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