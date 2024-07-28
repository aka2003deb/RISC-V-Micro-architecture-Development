/* Analysis of Components:
Program Counter (PC):
Holds the address of the current instruction.
Updated every clock cycle to point to the next instruction.

Instruction Memory:
Stores the instructions to be executed.
Outputs the current instruction based on the PC value.

Register File:
Contains the set of registers used by the instructions.
Two read ports (RD1, RD2) and one write port (WD3).
Inputs for register addresses (A1, A2, A3) and write enable (WE3).

ALU:
Performs arithmetic and logic operations.
Takes two operands (SrcA and SrcB) and outputs the result (ALUResult).

Data Memory:
Used for load/store instructions.
Takes an address (A), write data (WD), and write enable (WE).
Outputs read data (RD).

Multiplexers:
Used to select between different inputs based on control signals.

Sign Extend:
Extends the immediate value from the instruction to a 32-bit value.

Adder:
Calculates the next PC value (PC + 4).
Calculates the branch target address (PC + SignImm << 2) */


module Processor(
    input clk,
    input reset,
    output [31:0] result
);

    reg [31:0] PC;
    wire [31:0] Instr, ReadData1, ReadData2, ALUResult, ReadData, SignImm, PCBranch, PCPlus4;
    wire Zero;
    
    // PC logic
    always @(posedge clk or posedge reset) begin
        if (reset)
            PC <= 32'b0;
        else
            PC <= PCPlus4;
    end

    assign PCPlus4 = PC + 32'd4;

    // Instruction Memory
    InstructionMemory inst_mem (
        .A(PC),
        .RD(Instr)
    );

    // Register File
    RegisterFile reg_file (
        .clk(clk),
        .WE3(RegWrite),
        .A1(Instr[25:21]),
        .A2(Instr[20:16]),
        .A3(Instr[15:11]),
        .WD3(ALUResult),
        .RD1(ReadData1),
        .RD2(ReadData2)
    );

    // ALU
    ALU alu (
        .A(ReadData1),
        .B(ReadData2),
        .ALUResult(ALUResult),
        .Zero(Zero)
    );

    // Data Memory
    DataMemory data_mem (
        .clk(clk),
        .WE(MemWrite),
        .A(ALUResult),
        .WD(ReadData2),
        .RD(ReadData)
    );

    // Sign Extend
    assign SignImm = {{16{Instr[15]}}, Instr[15:0]};

    // Result
    assign result = ReadData;

endmodule

// Instruction Memory
module InstructionMemory (
    input [31:0] A,
    output [31:0] RD
);
    reg [31:0] memory [0:255];
    assign RD = memory[A >> 2];
endmodule

// Register File
module RegisterFile (
    input clk,
    input WE3,
    input [4:0] A1, A2, A3,
    input [31:0] WD3,
    output [31:0] RD1, RD2
);
    reg [31:0] regfile [0:31];
    assign RD1 = regfile[A1];
    assign RD2 = regfile[A2];
    always @(posedge clk) begin
        if (WE3)
            regfile[A3] <= WD3;
    end
endmodule

// ALU
module ALU (
    input [31:0] A, B,
    output [31:0] ALUResult,
    output Zero
);
    assign ALUResult = A + B;
    assign Zero = (ALUResult == 0);
endmodule

// Data Memory
module DataMemory (
    input clk,
    input WE,
    input [31:0] A, WD,
    output [31:0] RD
);
    reg [31:0] memory [0:255];
    assign RD = memory[A >> 2];
    always @(posedge clk) begin
        if (WE)
            memory[A >> 2] <= WD;
    end
endmodule
