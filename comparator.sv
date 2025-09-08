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

module comparator(
    input logic [31:0] rs1,
    input logic [31:0] rs2,
    output logic gt_u, eq_u, lt_u, gt_s, eq_s, lt_s
);
    always_comb begin
        gt_u = rs1 > rs2;
        eq_u = rs1 == rs2;
        lt_u = rs1 < rs2;
        gt_s = $signed(rs1) > $signed(rs2);
        eq_s = $signed(rs1) == $signed(rs2);
        lt_s = $signed(rs1) < $signed(rs2);
    end
endmodule
