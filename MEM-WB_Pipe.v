module MEM_WB_Pipeline(output reg [31:0]DataOut, output reg [31:0]NonLoadDataOut, output reg [3:0]Rd_Out, output reg LoadInst, output reg RF_EN, input [31:0]MuxLoadDataIn, input [31:0]MuxNonLoadDataIn, input [3:0]Rd_In, input LI, input RF_Enable)

always @(posedge clk)
begin
    DataOut <= MuxLoadDataIn;
    NonLoadDataOut <= MuxNonLoadDataIn;
    Rd_Out <= Rd_In;
    LoadInst <= LI;
    RF_EN <= RF_Enable;
end

endmodule