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

module alu(
    input logic [3:0] alu_op,
    input logic [31:0] a,
    input logic [31:0] b,
    output logic [31:0] result
);
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

    logic [4:0] shamt;
    assign shamt = b[4:0];

    always_comb begin
        result = 32'b0;
        unique case (alu_op)
            ADD: result = a + b;
            SUB: result = a - b;
            AND_: result = a & b;
            OR_: result = a | b;
            XOR_: result = a ^ b;
            SLT: result = ($signed(a) < $signed(b)) ? 32'b1 : 32'b0;
            SLTU: result = (a < b) ? 32'b1 : 32'b0;
            SLL: result = a << shamt;
            SRL: result = a >> shamt;
            SRA: result = $signed(a) >>> shamt;
            PASS_B: result = b;
            default: result = 32'b0;
        endcase
    end
endmodule
