// Copyright (C) 2025 Dongjun "Aurorasphere" Kim

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 3.

// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

`timescale 1ns / 1ps

module decoder(
    input logic [6:0] opcode,
    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic gt, eq, lt, lt_u, gt_u,  
    output logic MemWrite,
    output logic ALUSrc1,
    output logic ALUSrc2,
    output logic RegWrite,
    output logic MemExtType,
    output logic [3:0] ALUControl,
    output logic [2:0] ImmSrc,
    output logic [1:0] MemWriteSize,
    output logic [1:0] MemReadSize,
    output logic [1:0] PCSrc,
    output logic [1:0] WritebackSrc
);
    // Constants
    // Opcode encodings
    localparam logic [6:0] OPCODE_RTYPE = 7'b0110011;
    localparam logic [6:0] OPCODE_ITYPE = 7'b0010011;
    localparam logic [6:0] OPCODE_STYPE = 7'b0100011;
    localparam logic [6:0] OPCODE_ITYPE_LOAD = 7'b0000011;
    localparam logic [6:0] OPCODE_BTYPE = 7'b1100011;
    localparam logic [6:0] OPCODE_JALR = 7'b1100111;
    localparam logic [6:0] OPCODE_JAL = 7'b1101111;
    localparam logic [6:0] OPCODE_AUIPC = 7'b0010111;
    localparam logic [6:0] OPCODE_LUI = 7'b0110111;
    
    // Funct3 encodings
    localparam logic [2:0] FUNCT3_ADD_SUB = 3'b000;
    localparam logic [2:0] FUNCT3_SLL = 3'b001;
    localparam logic [2:0] FUNCT3_SLT = 3'b010;
    localparam logic [2:0] FUNCT3_SLTU = 3'b011;
    localparam logic [2:0] FUNCT3_XOR = 3'b100;
    localparam logic [2:0] FUNCT3_SRL_SRA = 3'b101;
    localparam logic [2:0] FUNCT3_OR = 3'b110;
    localparam logic [2:0] FUNCT3_AND = 3'b111;
    
    // Funct7 encodings
    localparam logic [6:0] FUNCT7_ADD = 7'b0000000;
    localparam logic [6:0] FUNCT7_SUB = 7'b0100000;
    localparam logic [6:0] FUNCT7_SRL = 7'b0000000;
    localparam logic [6:0] FUNCT7_SRA = 7'b0100000;
    
    // Branch condition encodings
    localparam logic [2:0] FUNCT3_BEQ = 3'b000;
    localparam logic [2:0] FUNCT3_BNE = 3'b001;
    localparam logic [2:0] FUNCT3_BLT = 3'b100;
    localparam logic [2:0] FUNCT3_BGE = 3'b101;
    localparam logic [2:0] FUNCT3_BLTU = 3'b110;
    localparam logic [2:0] FUNCT3_BGEU = 3'b111;
    
    // Memory size encodings
    localparam logic [1:0] MEM_SIZE_BYTE = 2'b00;
    localparam logic [1:0] MEM_SIZE_HALF = 2'b01;
    localparam logic [1:0] MEM_SIZE_WORD = 2'b10;
    
    // PC Source encodings
    localparam logic [1:0] PC_SRC_PC_PLUS_4 = 2'b00;
    localparam logic [1:0] PC_SRC_BRANCH = 2'b01;
    localparam logic [1:0] PC_SRC_ALU = 2'b10;
    
    // Writeback Source encodings
    localparam logic [1:0] WB_SRC_ALU = 2'b00;
    localparam logic [1:0] WB_SRC_MEM = 2'b01;
    localparam logic [1:0] WB_SRC_PC_PLUS_4 = 2'b10;
    
    // Immediate source encodings
    localparam logic [2:0] IMM_SRC_RTYPE_SHAMT = 3'b000;
    localparam logic [2:0] IMM_SRC_ITYPE = 3'b001;
    localparam logic [2:0] IMM_SRC_STYPE = 3'b010;
    localparam logic [2:0] IMM_SRC_BTYPE = 3'b011;
    localparam logic [2:0] IMM_SRC_JTYPE = 3'b100;
    localparam logic [2:0] IMM_SRC_UTYPE = 3'b101;

    localparam logic [3:0] ADD = 4'b0000;
    localparam logic [3:0] SUB = 4'b0001;
    localparam logic [3:0] AND_ = 4'b0010;
    localparam logic [3:0] OR_ = 4'b0011;
    localparam logic [3:0] XOR_ = 4'b0100;
    localparam logic [3:0] SLT = 4'b0101;
    localparam logic [3:0] SLTU = 4'b0110;
    localparam logic [3:0] SLL = 4'b0111;
    localparam logic [3:0] SRL = 4'b1000;
    localparam logic [3:0] SRA = 4'b1001;
    localparam logic [3:0] PASS_B = 4'b1010;
    
    always_comb begin 
        PCSrc = PC_SRC_PC_PLUS_4;
        MemWrite = 0;
        ALUSrc1 = 0;
        ALUSrc2 = 0;
        ImmSrc = IMM_SRC_RTYPE_SHAMT;
        RegWrite = 0;
        MemExtType = 0;
        ALUControl = 0;
        MemWriteSize = MEM_SIZE_BYTE;
        MemReadSize = MEM_SIZE_BYTE;
        WritebackSrc = WB_SRC_ALU;

        unique case (opcode)
            OPCODE_RTYPE: begin
                unique case (funct3)
                    FUNCT3_ADD_SUB: begin
                        unique case (funct7)
                            FUNCT7_ADD: begin
                                ALUControl = ADD;
                                RegWrite = 1;
                            end
                            FUNCT7_SUB: begin
                                ALUControl = SUB;
                                RegWrite = 1;
                            end
                            default: begin
                                ALUControl = 0;
                                RegWrite = 0;
                            end
                        endcase
                    end
                    FUNCT3_SLL: begin
                        unique case (funct7)
                            FUNCT7_ADD: begin
                                ALUControl = SLL;
                                RegWrite = 1;
                            end
                            default: begin
                                ALUControl = 0;
                                RegWrite = 0;
                            end
                        endcase
                    end
                    3'b010: begin
                        unique case (funct7)
                            7'b0000000: begin
                                ALUControl = SLT;
                                RegWrite = 1;
                            end
                            default: begin
                                ALUControl = 0;
                                RegWrite = 0;
                            end
                        endcase 
                    end
                    3'b011: begin
                        unique case (funct7)
                            7'b0000000: begin
                                ALUControl = SLTU;
                                RegWrite = 1;
                            end
                            default: begin
                                ALUControl = 0;
                                RegWrite = 0;
                            end
                        endcase 
                    end
                    3'b100: begin
                        unique case (funct7)
                            7'b0000000: begin
                                ALUControl = XOR_;
                                RegWrite = 1;
                            end
                            default: begin
                                ALUControl = 0;
                                RegWrite = 0;
                            end
                        endcase
                    end
                    3'b101: begin 
                        unique case (funct7)
                            7'b0000000: begin
                                ALUControl = SRL;
                                RegWrite = 1;
                            end
                            7'b0100000: begin
                                ALUControl = SRA;
                                RegWrite = 1;
                            end
                            default: begin
                                ALUControl = 0;
                                RegWrite = 0;
                            end
                        endcase
                    end
                    3'b110: begin
                        unique case (funct7)
                            7'b0000000: begin
                                ALUControl = OR_;
                                RegWrite = 1;
                            end
                            default: begin
                                ALUControl = 0;
                                RegWrite = 0;
                            end
                        endcase
                    end
                    3'b111: begin
                        unique case (funct7)
                            7'b0000000: begin
                                ALUControl = AND_;
                                RegWrite = 1;
                            end
                            default: begin
                                ALUControl = 0;
                                RegWrite = 0;
                            end
                        endcase
                    end
                endcase
            end
            OPCODE_ITYPE: begin
                unique case (funct3)
                    3'b000: begin
                        ALUSrc2 = 1;
                        RegWrite = 1;
                        ALUControl = ADD;
                        ImmSrc = IMM_SRC_ITYPE;
                        WritebackSrc = WB_SRC_ALU;
                    end
                    3'b001: begin
                        ALUControl = SLL;
                        ALUSrc2 = 1;
                        RegWrite = 1;
                        ImmSrc = IMM_SRC_RTYPE_SHAMT;
                        WritebackSrc = WB_SRC_ALU;
                    end
                    3'b010: begin
                        ALUControl = SLT;
                        ALUSrc2 = 1;
                        RegWrite = 1;
                        ImmSrc = IMM_SRC_ITYPE;
                        WritebackSrc = WB_SRC_ALU;
                    end
                    3'b011: begin
                        ALUControl = SLTU;
                        ALUSrc2 = 1;
                        RegWrite = 1;
                        ImmSrc = IMM_SRC_ITYPE;
                        WritebackSrc = WB_SRC_ALU;
                    end
                    3'b100: begin
                        ALUControl = XOR_;
                        ALUSrc2 = 1;
                        RegWrite = 1;
                        ImmSrc = IMM_SRC_ITYPE;
                        WritebackSrc = WB_SRC_ALU;
                    end
                    3'b101: begin
                        unique case (funct7)
                            7'b0000000: begin
                                ALUControl = SRL;
                                ALUSrc2 = 1;
                                RegWrite = 1;
                                ImmSrc = IMM_SRC_RTYPE_SHAMT;
                                WritebackSrc = WB_SRC_ALU;
                            end
                            7'b0100000: begin
                                ALUControl = SRA;
                                ALUSrc2 = 1;
                                RegWrite = 1;
                                ImmSrc = IMM_SRC_RTYPE_SHAMT;
                                WritebackSrc = WB_SRC_ALU;
                            end
                            default: begin
                                ALUControl = 0;
                                ALUSrc2 = 0;
                                RegWrite = 0;
                                WritebackSrc = WB_SRC_ALU;
                            end
                        endcase
                    end
                    3'b110: begin
                        ALUControl = OR_;
                        ALUSrc2 = 1;
                        RegWrite = 1;
                        ImmSrc = IMM_SRC_ITYPE;
                        WritebackSrc = WB_SRC_ALU;
                    end
                    3'b111: begin
                        ALUControl = AND_;
                        ALUSrc2 = 1;
                        RegWrite = 1;
                        ImmSrc = IMM_SRC_ITYPE;
                        WritebackSrc = WB_SRC_ALU;
                    end
                    default: begin
                        ALUControl = 0;
                        ALUSrc2 = 0;
                        RegWrite = 0;
                        ImmSrc = IMM_SRC_RTYPE_SHAMT;
                        WritebackSrc = WB_SRC_ALU;
                    end
                endcase
            end
            OPCODE_STYPE: begin
                unique case (funct3)
                    3'b000: begin
                        ALUSrc2 = 1;
                        ALUControl = ADD;
                        MemWrite = 1;
                        MemWriteSize = MEM_SIZE_BYTE;
                        ImmSrc = IMM_SRC_STYPE;
                    end
                    3'b001: begin
                        ALUSrc2 = 1;
                        ALUControl = ADD;
                        MemWrite = 1;
                        MemWriteSize = MEM_SIZE_HALF;
                        ImmSrc = IMM_SRC_STYPE;
                    end
                    3'b010: begin
                        ALUSrc2 = 1;
                        ALUControl = ADD;
                        MemWrite = 1;
                        MemWriteSize = MEM_SIZE_WORD;
                        ImmSrc = IMM_SRC_STYPE;
                    end
                    default: begin
                        ALUSrc2 = 0;
                        ALUControl = 0;
                        MemWrite = 0;
                        MemWriteSize = MEM_SIZE_BYTE;
                        ImmSrc = IMM_SRC_RTYPE_SHAMT;
                    end
                endcase 
            end
            OPCODE_ITYPE_LOAD: begin
                unique case (funct3)
                    3'b000: begin
                        MemReadSize = MEM_SIZE_BYTE;
                        ALUSrc2 = 1;
                        ALUControl = ADD;
                        MemExtType = 0;
                        RegWrite = 1;
                        WritebackSrc = WB_SRC_MEM;
                        ImmSrc = IMM_SRC_ITYPE;
                    end
                    3'b001: begin
                        MemReadSize = MEM_SIZE_HALF;
                        ALUSrc2 = 1;
                        ALUControl = ADD;
                        MemExtType = 0;
                        RegWrite = 1;
                        WritebackSrc = WB_SRC_MEM;
                        ImmSrc = IMM_SRC_ITYPE;
                    end
                    3'b010: begin  
                        MemReadSize = MEM_SIZE_WORD;
                        ALUSrc2 = 1;
                        ALUControl = ADD;
                        RegWrite = 1;
                        WritebackSrc = WB_SRC_MEM;
                        ImmSrc = IMM_SRC_ITYPE;
                    end
                    3'b100: begin
                        MemReadSize = MEM_SIZE_BYTE;
                        ALUSrc2 = 1;
                        ALUControl = ADD;
                        MemExtType = 1;
                        RegWrite = 1;
                        WritebackSrc = WB_SRC_MEM;
                        ImmSrc = IMM_SRC_ITYPE;
                    end
                    3'b101: begin
                        MemReadSize = MEM_SIZE_HALF;
                        ALUSrc2 = 1;
                        ALUControl = ADD;
                        MemExtType = 1;
                        RegWrite = 1;
                        WritebackSrc = WB_SRC_MEM;
                        ImmSrc = IMM_SRC_ITYPE;
                    end
                    default: begin
                        ALUSrc2 = 0;
                        MemReadSize = MEM_SIZE_BYTE;
                        ALUControl = 0;
                        MemExtType = 0;
                        WritebackSrc = WB_SRC_ALU;
                        RegWrite = 0;
                        ImmSrc = IMM_SRC_RTYPE_SHAMT;
                    end
                endcase
            end
            OPCODE_BTYPE: begin
                ImmSrc = IMM_SRC_BTYPE;
                unique case (funct3)
                    3'b000: begin
                        PCSrc = eq ? PC_SRC_BRANCH : PC_SRC_PC_PLUS_4;
                    end 
                    3'b001: begin
                        PCSrc = !eq ? PC_SRC_BRANCH : PC_SRC_PC_PLUS_4;
                    end
                    3'b100: begin
                        PCSrc = lt ? PC_SRC_BRANCH : PC_SRC_PC_PLUS_4;
                    end
                    3'b101: begin
                        PCSrc = (eq || gt) ? PC_SRC_BRANCH : PC_SRC_PC_PLUS_4;
                    end
                    3'b110: begin
                        PCSrc = lt_u ? PC_SRC_BRANCH : PC_SRC_PC_PLUS_4;
                    end
                    3'b111: begin
                        PCSrc = (eq || gt_u) ? PC_SRC_BRANCH : PC_SRC_PC_PLUS_4;
                    end
                    default: begin
                        PCSrc = PC_SRC_PC_PLUS_4;
                    end
                endcase
            end
            OPCODE_JALR: begin
                ALUSrc2 = 1;
                RegWrite = 1;
                ALUControl = ADD;
                PCSrc = PC_SRC_ALU;
                ImmSrc = IMM_SRC_ITYPE;
                WritebackSrc = WB_SRC_PC_PLUS_4;
            end
            OPCODE_JAL: begin
                PCSrc        = PC_SRC_BRANCH;
                ImmSrc       = IMM_SRC_JTYPE;
                WritebackSrc = WB_SRC_PC_PLUS_4;
                RegWrite     = 1;
            end
            OPCODE_AUIPC: begin
                ALUSrc1 = 1;
                ALUSrc2 = 1;
                ALUControl = ADD;
                RegWrite = 1;
                WritebackSrc = WB_SRC_ALU;
                ImmSrc = IMM_SRC_UTYPE;
            end
            OPCODE_LUI: begin
                ALUSrc2 = 1;
                ALUControl = PASS_B;
                RegWrite = 1;
                WritebackSrc = WB_SRC_ALU;
                ImmSrc = IMM_SRC_UTYPE;   // U-type immediate
            end
            default: begin
                PCSrc = 2'b00;
                ALUSrc1 = 0;
                ALUSrc2 = 0;
                ALUControl = 0;
                RegWrite = 0;
                ImmSrc = 3'b000;
                WritebackSrc = WB_SRC_ALU;
            end
            endcase
    end
endmodule