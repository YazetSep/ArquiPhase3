module ram256x32_inst(output reg [31:0]DataOut, input Enable, input [31:0]Instruction);

    reg [31:0]Mem[0:255];
    reg [7:0]Address = 8'b00000000;
    

    always @(*)
        if(Enable)
        begin
            Mem[Address] = Instruction;
            DataOut = Mem[Address]; //Always in Read Mode
            Address = Address + 1;
        end

endmodule

module ram256x32_data(output reg [31:0]DataOut, input Enable, ReadWrite, input[31:0]DataIn, input [31:0]Address, input [1:0]DataSize);

    reg [31:0]Mem[0:255];
    reg [31:0]DataTemp;
    always @(*)
    begin
        if(Enable)
            case (DataSize)
                //Byte
                2'b00    :  if (~ReadWrite) begin : named_block1 //Read - Load named_block:
                                integer i;
                                DataOut = 32'b0;
                                DataTemp = Mem[Address];
                                for(i = 0; i <= 7; i=i+1)
                                begin
                                    DataOut[i] = DataTemp[i];
                                end
                            end 
                            else 
                                begin : named_block2//Write - Store
                                    integer i;
                                    Mem[Address] = 32'b0;
                                    DataTemp = 32'b0;
                                    for(i = 0; i <= 7; i=i+1)
                                    begin
                                        DataTemp[i] = DataIn[i];
                                    end
                                    Mem[Address] = DataTemp;
                                end
                //Half-Word
                2'b01   :   if (!ReadWrite) begin : named_block3 //Read - Load
                                integer i;
                                DataOut = 32'b0;
                                DataTemp = Mem[Address];
                                for(i = 0; i <= 15; i=i+1)
                                begin
                                    DataOut[i] = DataTemp[i];
                                end
                            end 
                            else 
                                begin : named_block4 //Write - Store
                                    integer i;
                                    Mem[Address] = 32'b0;
                                    DataTemp = 32'b0;
                                    for(i = 0; i <= 15; i=i+1)
                                    begin
                                        DataTemp[i] = DataIn[i];
                                    end
                                    Mem[Address] = DataTemp;
                                end
                //Word
                2'b10   : if (!ReadWrite) 
                            begin //Read - Load
                                DataOut = Mem[Address];
                            end 
                            else 
                                begin //Write - Store
                                    Mem[Address] = DataIn;
                                end
                //Double Word
                2'b11   : if (!ReadWrite) 
                            begin //Read - Load
                                DataOut = Mem[Address];
                                #5;
                                DataOut = Mem[Address+1];
                            end 
                            else 
                                begin //Write - Store
                                    Mem[Address] = DataIn;
                                    #5;
                                    Mem[Address+1] = DataIn;
                                end
            endcase
    end
endmodule
