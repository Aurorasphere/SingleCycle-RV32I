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

module regfile #(
  parameter int XLEN=32, NREGS=32
)(
  input  logic clk, reset,
  input  logic we,
  input  logic [4:0] rs1, rs2, rd,
  input  logic [XLEN-1:0] wd,
  output logic [XLEN-1:0] rd1, rd2
);
  localparam logic [4:0] REG_X0 = 5'd0;
  logic [XLEN-1:0] regs [0:NREGS-1];

  function automatic logic [XLEN-1:0] rf_read(input logic [4:0] r);
    if (^r === 1'bX)        rf_read = '0;
    else if (r == REG_X0)   rf_read = '0;
    else                    rf_read = regs[r];
  endfunction

  // Synchronous write with x0 forced to zero
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      for (int i=0;i<NREGS;i++) regs[i] <= '0;
    end else begin
      if (we && (rd != REG_X0)) regs[rd] <= wd;
      regs[REG_X0] <= '0;
    end
  end

  // Forwarding with 1-cycle delayed information only
  logic        we_q;
  logic [4:0]  rd_q;
  logic [XLEN-1:0] wd_q;
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      we_q <= 1'b0; rd_q <= REG_X0; wd_q <= '0;
    end else begin
      we_q <= (we === 1'b1) && (rd != REG_X0);
      rd_q <= rd;
      wd_q <= wd;
    end
  end

  wire hit1 = we_q && (rd_q == rs1);
  wire hit2 = we_q && (rd_q == rs2);
  assign rd1 = hit1 ? wd_q : rf_read(rs1);
  assign rd2 = hit2 ? wd_q : rf_read(rs2);

endmodule