module control_unit(output reg [3:0] ID_ALU_OP, output reg [1:0] data_size, output reg ID_shift_imm, ID_load_instr, 
        ID_RF_enable, ID_B_instr, RW, input [31:0] instruction);
    always @ (instruction)
    begin
        ID_ALU_OP = instruction[24:21];
        ID_shift_imm = instruction[25];
        ID_load_instr = ((instruction[27:25] === 3'b010) | (instruction[27:25] === 3'b011)) & instruction[20];
        ID_RF_enable = (instruction[15:12] == 1'b1)     // If there is a destination register
        ID_B_instr = (instruction[27:25] === 3'b101);
        // Verify
        RW = ((ID_load_instr) | ~(ID_RF_enable));
        // TODO
        data_size = 1'b00;
        //
    end
endmodule