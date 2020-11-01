module x4_Sign_Extender(output reg [31:0] extended, input [23:0] input_instr);
    always @ (*) begin
        extended = {{8{1'b0}}, input_instr};
        extended = extended * 4;
    end
endmodule

//Test
module x4_Sign_Extender_test;
    //input
    reg [23:0] input_instr;
    
    //output
    wire [31:0] extended;
    
    //Intantiation
    x4_Sign_Extender unit (extended, input_instr);
    
    initial begin
        input_instr = $random;
    end
    
    initial begin
        $display ("Input(binary)                  Input(decimal)        Extended Output(binary)            Extended Output (decimal)");
        $monitor ("%b      %d          %b       %d", input_instr, input_instr, extended, extended);
    end
endmodule