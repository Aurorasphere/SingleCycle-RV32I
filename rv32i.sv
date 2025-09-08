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

module rv32i #(
    parameter int IMEM_AW = 12,
    parameter int DMEM_AW = 12
)(
    input  logic        clk,
    input  logic        reset,
    output logic [31:0] output_value,

    // debug
    output logic [1:0]  dbg_PCSrc,
    output logic [31:0] dbg_pc,
    output logic [31:0] dbg_pc_next,
    output logic [31:0] dbg_instr,
    output logic        dbg_rs1_src,
    output logic        dbg_rs2_src,
    output logic [4:0]  dbg_rs1_sel,
    output logic [4:0]  dbg_rs2_sel,
    output logic [4:0]  dbg_rd_sel,
    output logic [31:0] dbg_rs1_data, 
    output logic [31:0] dbg_rs2_data,
    output logic [31:0] dbg_imm_value,
    output logic [3:0]  dbg_ALU_control,
    output logic [31:0] dbg_ALU_a_data,
    output logic [31:0] dbg_ALU_b_data,
    output logic [31:0] dbg_ALU_result,
    output logic [31:0] dbg_load_data,
    output logic [31:0] dbg_write_data,
    output logic [1:0]  dbg_WritebackSrc,
    output logic [31:0] dbg_writeback_data
);
    // constants
    localparam logic [31:0] PC_INIT       = 32'h0000_0000;
    localparam logic [31:0] PC_INC        = 32'd4;
    localparam logic [31:0] MMIO_OUT_ADDR = 32'h4000_0000;
    localparam logic [31:0] MMIO_MASK     = 32'hFFFF_FFFC;

    // PC src
    localparam logic [1:0] PC_SRC_PC_PLUS_4 = 2'b00;
    localparam logic [1:0] PC_SRC_BRANCH    = 2'b01;
    localparam logic [1:0] PC_SRC_ALU       = 2'b10;

    // WB src
    localparam logic [1:0] WB_SRC_ALU       = 2'b00;
    localparam logic [1:0] WB_SRC_MEM       = 2'b01;
    localparam logic [1:0] WB_SRC_PC_PLUS_4 = 2'b10;

    // mem size
    localparam logic [1:0] MEM_SIZE_BYTE = 2'b00;
    localparam logic [1:0] MEM_SIZE_HALF = 2'b01;
    localparam logic [1:0] MEM_SIZE_WORD = 2'b10;

    // imm src
    localparam logic [2:0] IMM_SRC_RTYPE_SHAMT = 3'b000;
    localparam logic [2:0] IMM_SRC_ITYPE       = 3'b001;
    localparam logic [2:0] IMM_SRC_STYPE       = 3'b010;
    localparam logic [2:0] IMM_SRC_BTYPE       = 3'b011;
    localparam logic [2:0] IMM_SRC_JTYPE       = 3'b100;
    localparam logic [2:0] IMM_SRC_UTYPE       = 3'b101;

    // control
    logic [1:0] PCSrc;
    logic       MemWrite, ALUSrc1, ALUSrc2, RegWrite, MemExtType;
    logic [2:0] ImmSrc;
    logic [1:0] MemWriteSize, MemReadSize, WritebackSrc;
    logic [3:0] ALUControl;

    // compare
    logic gt_u, eq_u, lt_u, gt_s, eq_s, lt_s;

    // dmem
    logic [31:0] dmem_rd;

    // PC path
    logic [31:0] pc, pc_plus_4, next_pc, pc_branch, pc_jalr;
    logic [31:0] pc_fetch, pc_plus_4_fetch, pc_branch_fetch;

    // ALU path
    logic [31:0] ALU_a_data, ALU_b_data, ALUResult;

    // regs
    logic [31:0] rs1_data, rs2_data, write_data;

    // instr
    logic [31:0] instruction_raw;
    logic [31:0] instr_q;

    // imem
    imem #(.ADDR_WIDTH(IMEM_AW)) imem_inst(
        .addr(pc[IMEM_AW-1:0]),
        .data(instruction_raw)
    );

    // fetch-stage latches: instr and its PC
    always_ff @(posedge clk or posedge reset) begin 
        if (reset) begin
            instr_q  <= 32'h0000_0013;
            pc_fetch <= PC_INIT;
        end else begin
            instr_q  <= instruction_raw;
            pc_fetch <= pc; // PC of this instruction
        end
    end

    // immediates
    logic signed [31:0] imm_sel;
    logic [31:0] rtype_shamt, imm_i_ext, imm_s_ext, imm_b_ext, imm_u_ext, imm_j_ext;

    always_comb begin
        rtype_shamt = {27'd0, instr_q[24:20]};
        imm_i_ext   = {{20{instr_q[31]}}, instr_q[31:20]};
        imm_s_ext   = {{20{instr_q[31]}}, instr_q[31:25], instr_q[11:7]};
        imm_b_ext   = {{19{instr_q[31]}}, instr_q[31], instr_q[7],
                       instr_q[30:25], instr_q[11:8], 1'b0};
        imm_u_ext   = {instr_q[31:12], 12'b0};
        imm_j_ext   = {{11{instr_q[31]}}, instr_q[31], instr_q[19:12],
                       instr_q[20], instr_q[30:21], 1'b0};
    end

    always_comb begin
        unique case (ImmSrc)
            IMM_SRC_RTYPE_SHAMT: imm_sel = rtype_shamt;
            IMM_SRC_ITYPE      : imm_sel = imm_i_ext;
            IMM_SRC_STYPE      : imm_sel = imm_s_ext;
            IMM_SRC_BTYPE      : imm_sel = imm_b_ext;
            IMM_SRC_JTYPE      : imm_sel = imm_j_ext;
            IMM_SRC_UTYPE      : imm_sel = imm_u_ext;
            default            : imm_sel = 32'sd0;
        endcase
    end

    // MMIO out
    logic [31:0] mmio_out_reg;

    // write-path latches
    logic        mem_we_q;
    logic [1:0]  mem_wsize_q;
    logic [31:0] mem_addr_q, mem_wdata_q;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_we_q     <= 1'b0;
            mem_wsize_q  <= 2'b00;
            mem_addr_q   <= 32'b0;
            mem_wdata_q  <= 32'b0;
        end else begin
            mem_we_q     <= MemWrite;
            mem_wsize_q  <= MemWriteSize;
            mem_addr_q   <= ALUResult;
            mem_wdata_q  <= rs2_data;
        end
    end

    // MMIO write decode and DMEM gate
    logic is_mmio_out_w;
    assign is_mmio_out_w = ((mem_addr_q & MMIO_MASK) == MMIO_OUT_ADDR);

    // DMEM address bus
    logic [31:0] dmem_addr_bus;
    assign dmem_addr_bus = mem_we_q ? mem_addr_q : ALUResult;

    // DMEM write enable
    logic dmem_we;
    assign dmem_we = mem_we_q & ~is_mmio_out_w;

    // dmem
    dmem #(.ADDR_WIDTH(DMEM_AW), .DMEM_FILE("dmem.hex")) dmem_inst(
        .clk(clk),
        .we(dmem_we),
        .write_size(mem_wsize_q),
        .read_size(MemReadSize),
        .unsigned_extend(MemExtType),
        .addr(dmem_addr_bus[DMEM_AW-1:0]),
        .wd(mem_wdata_q),
        .rd(dmem_rd)
    );

    // MMIO write strobes
    logic [3:0] io_wstrb;
    always_comb begin
        unique case (mem_wsize_q)
            MEM_SIZE_BYTE: io_wstrb = 4'b0001 << mem_addr_q[1:0];
            MEM_SIZE_HALF: io_wstrb = (mem_addr_q[1] ? 4'b1100 : 4'b0011);
            default      : io_wstrb = 4'b1111;
        endcase
    end

    always_ff @(posedge clk or posedge reset) begin
        if (reset) mmio_out_reg <= 32'h0;
        else if (mem_we_q && is_mmio_out_w) begin
            if (io_wstrb[0]) mmio_out_reg[7:0]   <= mem_wdata_q[7:0];
            if (io_wstrb[1]) mmio_out_reg[15:8]  <= mem_wdata_q[15:8];
            if (io_wstrb[2]) mmio_out_reg[23:16] <= mem_wdata_q[23:16];
            if (io_wstrb[3]) mmio_out_reg[31:24] <= mem_wdata_q[31:24];
        end
    end

    // MMIO read path
    logic is_mmio_out_r;
    assign is_mmio_out_r = ((ALUResult & MMIO_MASK) == MMIO_OUT_ADDR);

    logic [31:0] mmio_rd, mmio_word;
    logic [7:0]  io_byte;
    logic [15:0] io_half;

    assign mmio_word = mmio_out_reg;

    always_comb begin
        unique case (MemReadSize)
            MEM_SIZE_BYTE: begin
                io_byte = mmio_word >> (8*ALUResult[1:0]);
                mmio_rd = MemExtType ? {24'h0, io_byte} : {{24{io_byte[7]}}, io_byte};
            end
            MEM_SIZE_HALF: begin
                io_half = mmio_word >> (16*ALUResult[1]);
                mmio_rd = MemExtType ? {16'h0, io_half} : {{16{io_half[15]}}, io_half};
            end
            MEM_SIZE_WORD: mmio_rd = mmio_word;
            default      : mmio_rd = 32'h0;
        endcase
    end

    // load mux
    logic [31:0] load_data;
    assign load_data = is_mmio_out_r ? mmio_rd : dmem_rd;

    // regfile
    regfile regfile_inst(
        .clk(clk),
        .reset(reset),
        .we(RegWrite),
        .rs1(instr_q[19:15]),
        .rs2(instr_q[24:20]),
        .rd (instr_q[11:7]),
        .wd (write_data),
        .rd1(rs1_data),
        .rd2(rs2_data)
    );

    // decoder
    decoder decoder_inst(
        .opcode(instr_q[6:0]),
        .funct3(instr_q[14:12]),
        .funct7(instr_q[31:25]),
        .gt(gt_s),
        .eq(eq_s),
        .lt(lt_s),
        .lt_u(lt_u),
        .gt_u(gt_u),
        .MemWrite(MemWrite),
        .ALUSrc1(ALUSrc1),
        .ALUSrc2(ALUSrc2),
        .RegWrite(RegWrite),
        .MemExtType(MemExtType),
        .ALUControl(ALUControl),
        .ImmSrc(ImmSrc),
        .MemWriteSize(MemWriteSize),
        .MemReadSize(MemReadSize),
        .PCSrc(PCSrc),
        .WritebackSrc(WritebackSrc)
    );

    // comparator
    comparator cmp_inst(
        .rs1(rs1_data), .rs2(rs2_data),
        .gt_u(gt_u), .eq_u(eq_u), .lt_u(lt_u),
        .gt_s(gt_s), .eq_s(eq_s), .lt_s(lt_s)
    );

    // ALU inputs
    always_comb begin
        ALU_a_data = ALUSrc1 ? pc      : rs1_data; // AUIPC: pc used here
        ALU_b_data = ALUSrc2 ? imm_sel : rs2_data;
    end

    // ALU
    alu alu_inst(
        .alu_op(ALUControl),
        .a(ALU_a_data),
        .b(ALU_b_data),
        .result(ALUResult)
    );

    // next PC (fetch-PC based)
    always_comb begin
        pc_plus_4        = pc + PC_INC;          // for debug only
        pc_branch        = pc + imm_sel;         // for debug only

        pc_plus_4_fetch  = pc_fetch + PC_INC;    // correct link value
        pc_branch_fetch  = pc_fetch + imm_sel;   // branch/jal base
        pc_jalr          = {ALUResult[31:1], 1'b0};

        unique case (PCSrc)
            PC_SRC_PC_PLUS_4: next_pc = pc_plus_4_fetch;
            PC_SRC_BRANCH   : next_pc = pc_branch_fetch;
            PC_SRC_ALU      : next_pc = pc_jalr;
            default         : next_pc = pc_plus_4_fetch;
        endcase
    end

    // writeback
    always_comb begin
        unique case (WritebackSrc)
            WB_SRC_ALU      : write_data = ALUResult;
            WB_SRC_MEM      : write_data = load_data;
            WB_SRC_PC_PLUS_4: write_data = pc_plus_4_fetch; // crucial
            default         : write_data = ALUResult;
        endcase
    end

    // PC register (negedge as required)
    always_ff @(negedge clk or posedge reset) begin
        if (reset) pc <= PC_INIT;
        else       pc <= next_pc;
    end

    // output
    assign output_value = mmio_out_reg;

    // debug
    assign dbg_PCSrc          = PCSrc;
    assign dbg_pc             = pc;
    assign dbg_pc_next        = next_pc;
    assign dbg_instr          = instr_q;
    assign dbg_rs1_src        = ALUSrc1;
    assign dbg_rs2_src        = ALUSrc2;
    assign dbg_rs1_sel        = instr_q[19:15];
    assign dbg_rs2_sel        = instr_q[24:20];
    assign dbg_rd_sel         = instr_q[11:7];
    assign dbg_rs1_data       = rs1_data;
    assign dbg_rs2_data       = rs2_data;
    assign dbg_imm_value      = imm_sel;
    assign dbg_ALU_control    = ALUControl;
    assign dbg_ALU_a_data     = ALU_a_data;
    assign dbg_ALU_b_data     = ALU_b_data;
    assign dbg_ALU_result     = ALUResult;
    assign dbg_load_data      = load_data;
    assign dbg_write_data     = write_data;
    assign dbg_WritebackSrc   = WritebackSrc;
    assign dbg_writeback_data = write_data;
endmodule