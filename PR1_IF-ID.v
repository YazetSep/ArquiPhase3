module IFID (output reg [31:0] PC_out, instr_out, input [31:0] PC_in, instr_in, input clk, reset, LE);
always @(posedge clk, posedge reset) begin
    if (reset)
        instr_out <= 0; //Control hazard handling reset
    else if (LE) begin //LE is used to stall when load hazard is asserted from the Haz/Forw Unit
        PC_out <= PC_in;
        instr_out <= instr_in;
    end
end
endmodule