# Single Cycle RISC-V Processor

## Overview

This project is a 32-bit single-cycle RISC-V processor implemented in SystemVerilog. It supports the RV32I base integer instruction set and is designed for FPGA implementation in the Xilinx Vivado environment.

## Project Structure

```
Single Cycle RISC-V/
├── rv32i_core/                 # Core processor modules
│   ├── rv32i.sv               # Main processor module
│   ├── alu.sv                 # Arithmetic Logic Unit (ALU)
│   ├── comparator.sv          # Comparator module
│   ├── decoder.sv             # Instruction decoder
│   ├── memory.sv              # Memory system (IMEM/DMEM)
│   ├── regfile.sv             # Register file
│   ├── imem.hex               # Instruction memory initialization file
│   └── dmem.hex               # Data memory initialization file
└── tb_rv32i_top.sv            # Testbench
```

## Key Features

### Supported Instructions
- **R-Type**: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
- **I-Type**: ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU
- **Load**: LB, LH, LW, LBU, LHU
- **Store**: SB, SH, SW
- **Branch**: BEQ, BNE, BLT, BGE, BLTU, BGEU
- **Jump**: JAL, JALR
- **Upper**: LUI, AUIPC

### Architecture Features
- **Single-cycle implementation**: All instructions complete in one clock cycle
- **Harvard architecture**: Separate instruction memory (IMEM) and data memory (DMEM)
- **32 general-purpose registers**: x0~x31 (x0 is always hardwired to zero)
- **Memory-mapped I/O**: Output through MMIO address `0x4000_0000`
- **Debug interface**: Debug ports for internal signal monitoring

### Memory System
- **IMEM**: 12-bit address (4KB), 32-bit word access
- **DMEM**: 12-bit address (4KB), byte/halfword/word access
- **Little-endian** byte order
- **Distributed memory** style implementation (utilizing FPGA LUTs)

## Module Description

### 1. `rv32i.sv` - Main Processor
- Top-level module integrating all sub-modules
- Pipeline latch and control signal management
- MMIO output handling
- Extensive debug signal provision

### 2. `alu.sv` - Arithmetic Logic Unit
- Supports 10 operations (ADD, SUB, AND, OR, XOR, SLT, SLTU, SLL, SRL, SRA)
- 32-bit signed/unsigned arithmetic
- 5-bit shift amount support for shift operations

### 3. `decoder.sv` - Instruction Decoder
- Instruction decoding based on opcode, funct3, funct7
- Generation of all control signals
- Branch condition handling
- Memory access control

### 4. `regfile.sv` - Register File
- 32 32-bit registers
- Simultaneous 2-port read, 1-port write
- x0 register hardwired to zero
- Data forwarding support

### 5. `memory.sv` - Memory System
- **IMEM**: Instruction memory, ROM-style
- **DMEM**: Data memory, RAM-style
- Byte/halfword/word access support
- Sign extension/zero extension support

### 6. `comparator.sv` - Comparator
- Comparison operations for branch instructions
- Signed/unsigned comparison support
- 6 types of comparison result outputs

## Memory Initialization

### Instruction Memory (`imem.hex`)
- Store RISC-V assembly converted to machine code
- Stored in hexadecimal format, byte-wise
- Default: NOP instruction (0x00000013)

### Data Memory (`dmem.hex`)
- Set initial data values
- Default: All memory locations initialized to 0x00

## MMIO (Memory-Mapped I/O)

- **Output Register**: Address `0x4000_0000`
- Output to external through processor's `output_value` port
- Supports byte/halfword/word write operations
- Read support for checking current output value

## Debug Features

The processor provides the following debug signals:
- PC related: Current PC, next PC, PC source
- Instruction: Currently executing instruction
- Registers: rs1, rs2, rd selection and data
- ALU: Input, output, control signals
- Memory: Load/store data
- Control: Writeback source and data

## License

This project is distributed under the GNU General Public License v3.0.

```
Copyright (C) 2025 Dongjun "Aurorasphere" Kim

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.
```

## Notes

- This implementation is designed for educational purposes
- Uses distributed memory style for FPGA resource efficiency
- Single-cycle implementation focuses on simplicity over performance
- Tested with Vivado 2020.1 or later versions

## Future Improvements

- Pipeline implementation for performance enhancement
- Cache system addition
- Support for more RISC-V extensions (M, C, etc.)
- Advanced debug interface implementation

