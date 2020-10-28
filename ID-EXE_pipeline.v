module ID_EXE_pipeline(output reg [31:0] EXE_In, EXE_A, EXE_B, output reg [11:0] EXE_immed, output reg [4:0] EXE_Opcode, output reg [3:0] EXE_Rd_num, output reg [2:0] EXE_I_cmd, output reg EXE_S, 
input [31:0] ID_A, ID_B, input [11:0] ID_immed, input [4:0] ID_Opcode, input [3:0] ID_Rd_num, input [2:0] ID_I_cmd, input ID_S, Clk);
    always @ (posedge Clk)
    begin
        EXE_In = ID_A;
        EXE_A = ID_A;
        EXE_B = ID_B;
        EXE_immed = ID_immed;
        EXE_Opcode = ID_Opcode;
        EXE_Rd_num = ID_Rd_num;
        EXE_I_cmd = ID_I_cmd;
        EXE_S = ID_S;
    end
endmodule 