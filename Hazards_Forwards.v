module HazForwUnit (output reg [1:0] mux_S_A, mux_S_B, input [3:0] EX_Rd, MEM_Rd, WB_Rd, ID_Rn, ID_Rm, input EX_RF_enable, MEM_RF_enable, WB_RF_enable);
/*Mux A and Mux B Cheat Sheet:
    00 -> PA/PB Original Register File Outputs
    01 -> [EX_Rd]
    10 -> [MEM_Rd]
    11 -> [WB_Rd]
Note: The multiplexers are mirrored vertically. Top port of Mux A is selected with 11 while top port of Mux B is selected with 00.
*/
always @ (*) begin
    //TODO Special Forwarding Cases: Forwarding is only allowed from the stage closest to ID

    //EX forwarding
    if (EX_RF_enable && (ID_Rn === EX_Rd))
        mux_S_A = 2'b01; //Forwarding [EX_Rd] to ID
    else mux_S_A = 2'b00; //Not forwarding (passing PA from the register file)
    
    if (EX_RF_enable && (ID_Rm === EX_Rd))
        mux_S_B = 2'b01; //Forwarding [EX_Rd] to ID
    else mux_S_B = 2'b00; //Not forwarding (passing PB from the register file)

    //MEM forwarding
    if (MEM_RF_enable && (ID_Rn === MEM_Rd))
        mux_S_A = 2'b10; //Forwarding [MEM_Rd] to ID
    else mux_S_A = 2'b00; //Not forwarding (passing PA from the register file)
    
    if (MEM_RF_enable && (ID_Rm === MEM_Rd))
        mux_S_B = 2'b10; //Forwarding [MEM_Rd] to ID
    else mux_S_B = 2'b00; //Not forwarding (passing PB from the register file)

    //WB forwarding
    if (WB_RF_enable && (ID_Rn === WB_Rd))
        mux_S_A = 2'b11; //Forwarding [WB_Rd] to ID
    else mux_S_A = 2'b00; //Not forwarding (passing PA from the register file)
    
    if (WB_RF_enable && (ID_Rm === WB_Rd))
        mux_S_B = 2'b11; //Forwarding [WB_Rd] to ID
    else mux_S_B = 2'b00; //Not forwarding (passing PB from the register file)
end
endmodule