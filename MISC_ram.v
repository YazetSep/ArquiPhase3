module ram256x32_inst(output reg [31:0]DataOut, input Enable, input [31:0]Instruction);

    reg [31:0]Mem[0:255];
    reg [7:0]Address;
    Address = 8'b00000000

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

    always @(*)
    begin
        if(Enable)
            begin
            reg [31:0]DataTemp;
            case (DataSize)
                //Byte
                2'b00    :  if (!ReadWrite) 
                            begin //Read - Load
                                integer i;
                                DataOut = 31'b00000000000000000000000000000000;
                                DataTemp = Mem[Address]
                                for(i = 0; i < 8; i=i+1)
                                begin
                                    DataOut[i] = DataTemp[i];
                                end
                            end 
                            else 
                                begin //Write - Store
                                    Mem[Address] = 31'b00000000000000000000000000000000;
                                    DataTemp = 31'b00000000000000000000000000000000;
                                    integer i;
                                    for(i = 0; i < 8; i=i+1)
                                    begin
                                        DataTemp[i] = DataIn[i];
                                    end
                                    Mem[Address] = DataTemp;
                                end
                //Half-Word
                2'b01   :   if (!ReadWrite) 
                            begin //Read - Load
                                integer i;
                                DataOut = 31'b00000000000000000000000000000000;
                                DataTemp = Mem[Address]
                                for(i = 0; i < 15; i=i+1)
                                begin
                                    DataOut[i] = DataTemp[i];
                                end
                            end 
                            else 
                                begin //Write - Store
                                    Mem[Address] = 31'b00000000000000000000000000000000;
                                    DataTemp = 31'b00000000000000000000000000000000;
                                    integer i;
                                    for(i = 0; i < 8; i=i+1)
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
                2b'11   : if (!ReadWrite) 
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
    end
endmodule
