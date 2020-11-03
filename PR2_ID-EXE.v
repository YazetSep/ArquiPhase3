module ID_EXE_pipeline(output reg [31:0] EXE_In, EXE_A, EXE_B, output reg [11:0] EXE_immed, output reg [4:0] EXE_Opcode, output reg [3:0] EXE_Rd_num, EXE_ALU_OP, output reg [2:0] EXE_I_cmd, 
output reg EXE_S, EXE_shift_imm, EXE_load_instr, EXE_RF_enable, input [31:0] ID_A, ID_B, instruction, input[3:0] ID_ALU_OP, input ID_shift_imm, ID_load_instr, ID_RF_enable, clk);
    always @ (posedge clk)
    begin
        EXE_In <= ID_A;
        EXE_A <= ID_A;
        EXE_B <= ID_B;
        EXE_immed <= instruction[11:0];
        EXE_Opcode <= instruction[24:20];
        EXE_Rd_num <= instruction[15:12];
        EXE_I_cmd <= instruction[27:25];
        EXE_S <= instruction[20];
        EXE_shift_imm <= ID_shift_imm;
        EXE_ALU_OP <= ID_ALU_OP;
        EXE_load_instr <= ID_load_instr;
        EXE_RF_enable <= ID_RF_enable;
    end
endmodule 