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

`timescale 1ns/1ps

// Instruction Memory (IMEM)
// Byte-addressed, little-endian, LUT ROM
module imem #(
    parameter int ADDR_WIDTH = 12,
    parameter string ROM_FILE = "imem.hex"  
)(
    input  logic [ADDR_WIDTH-1:0] addr,
    output logic [31:0]           data
);
    // Constants
    localparam logic [7:0] NOP_BYTE_0 = 8'h13;
    localparam logic [7:0] NOP_BYTE_1 = 8'h00;
    localparam logic [7:0] NOP_BYTE_2 = 8'h00;
    localparam logic [7:0] NOP_BYTE_3 = 8'h00;
    localparam logic [7:0] MEM_INIT_VALUE = 8'h00;
    localparam logic [1:0] WORD_ALIGN_MASK = 2'b00;
    (* rom_style = "distributed" *)
    logic [7:0] rom [0:(1<<ADDR_WIDTH)-1];


    initial begin
        for (int i = 0; i < (1<<ADDR_WIDTH); i += 4) begin
            rom[i+0] = NOP_BYTE_0;
            rom[i+1] = NOP_BYTE_1;
            rom[i+2] = NOP_BYTE_2;
            rom[i+3] = NOP_BYTE_3;
        end

        $readmemh(ROM_FILE, rom);
    end

    // 32-bit fetch (little-endian)
    wire [ADDR_WIDTH-1:0] base = {addr[ADDR_WIDTH-1:2], WORD_ALIGN_MASK};
    assign data = {rom[base+3], rom[base+2], rom[base+1], rom[base+0]};

    // synopsys translate_off
    always @* if (addr[1:0] != WORD_ALIGN_MASK)
        $warning("imem: unaligned fetch addr=%0h", addr);
    // synopsys translate_on
endmodule


// Data Memory (DMEM)
// Byte-addressed, little-endian, LUT RAM
module dmem #(
    parameter int ADDR_WIDTH = 12,
    parameter string DMEM_FILE = "dmem.hex"
)(
    input  logic                  clk,
    input  logic                  we,
    input  logic [1:0]            write_size,
    input  logic [1:0]            read_size,
    input  logic                  unsigned_extend,
    input  logic [ADDR_WIDTH-1:0] addr,
    input  logic [31:0]           wd,
    output logic [31:0]           rd
);
    // Constants
    localparam logic [7:0] MEM_INIT_VALUE = 8'h00;
    localparam logic [1:0] WORD_ALIGN_MASK = 2'b00;
    localparam logic [1:0] HALFWORD_ALIGN_MASK = 2'b01;
    
    // Memory size encodings
    localparam logic [1:0] MEM_SIZE_BYTE = 2'b00;
    localparam logic [1:0] MEM_SIZE_HALF = 2'b01;
    localparam logic [1:0] MEM_SIZE_WORD = 2'b10;
    
    // Write strobe patterns
    localparam logic [3:0] WSTRB_BYTE_LOW = 4'b0001;
    localparam logic [3:0] WSTRB_BYTE_HIGH = 4'b1000;
    localparam logic [3:0] WSTRB_HALF_LOW = 4'b0011;
    localparam logic [3:0] WSTRB_HALF_HIGH = 4'b1100;
    localparam logic [3:0] WSTRB_WORD = 4'b1111;
    localparam logic [3:0] WSTRB_NONE = 4'b0000;
    (* ram_style = "distributed" *)
    logic [7:0] ram [0:(1<<ADDR_WIDTH)-1];

    initial begin
        // Initialize all memory to zero first
        for (int i = 0; i < (1<<ADDR_WIDTH); i++) ram[i] = MEM_INIT_VALUE;
        
        // Try to load dmem.hex file if it exists
        $readmemh(DMEM_FILE, ram);
    end

    logic [3:0] wstrb;
    always @* begin
        unique case (write_size)
            MEM_SIZE_BYTE: wstrb = WSTRB_BYTE_LOW << addr[1:0];
            MEM_SIZE_HALF: wstrb = (addr[1] ? WSTRB_HALF_HIGH : WSTRB_HALF_LOW);
            MEM_SIZE_WORD: wstrb = WSTRB_WORD; 
            default: wstrb = WSTRB_NONE;
        endcase
    end

    // synopsys translate_off
    always @* begin
        if (we && write_size==MEM_SIZE_HALF && addr[0])
            $warning("dmem: unaligned halfword store addr=%0h", addr);
        if (we && write_size==MEM_SIZE_WORD && addr[1:0]!=WORD_ALIGN_MASK)
            $warning("dmem: unaligned word store addr=%0h", addr);
        if (read_size==MEM_SIZE_HALF && addr[0])
            $warning("dmem: unaligned halfword load addr=%0h", addr);
        if (read_size==MEM_SIZE_WORD && addr[1:0]!=WORD_ALIGN_MASK)
            $warning("dmem: unaligned word load addr=%0h", addr);
    end
    // synopsys translate_on

    wire [ADDR_WIDTH-1:0] base = {addr[ADDR_WIDTH-1:2], WORD_ALIGN_MASK};

    always_ff @(posedge clk) if (we) begin
        if (wstrb[0]) ram[base+0] <= wd[7:0];
        if (wstrb[1]) ram[base+1] <= wd[15:8];
        if (wstrb[2]) ram[base+2] <= wd[23:16];
        if (wstrb[3]) ram[base+3] <= wd[31:24];
    end

    wire [31:0] word = {ram[base+3], ram[base+2], ram[base+1], ram[base+0]};
    logic [7:0]  byte_data;
    logic [15:0] half_data;

    always_comb begin
        unique case (read_size)
            MEM_SIZE_BYTE: begin
                byte_data = word >> (8*addr[1:0]);
                rd = unsigned_extend ? {24'h0, byte_data}
                                      : {{24{byte_data[7]}}, byte_data};
            end
            MEM_SIZE_HALF: begin
                half_data = word >> (16*addr[1]);
                rd = unsigned_extend ? {16'h0, half_data}
                                      : {{16{half_data[15]}}, half_data};
            end
            MEM_SIZE_WORD: begin
                rd = word;
            end
            default: rd = 32'h0;
        endcase
    end
endmodule
