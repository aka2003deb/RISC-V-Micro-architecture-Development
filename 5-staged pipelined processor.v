/*
This Verilog code is a basic implementation of a 5-stage pipelined processor for the RV32I ISA.
This implementation needs to be refined further to handle pipeline hazards, forwarding, and control signals correctly.
The instruction set and memory modules are simplified for illustration purposes and should be expanded to cover the complete RV32I ISA.
The code assumes an ideal memory model and register file. 
You should integrate actual memory and register file modules that can handle read and write operations properly  
*/

module PipelinedRiscVProcessor (
    input clk,
    input reset
);
    // Wires for Fetch Stage
    wire [31:0] PCF, InstrF;

    // Wires for Decode Stage
    wire [31:0] PCPlus4D, InstrD;
    wire [31:0] SrcAE, SrcBE, SignImmE, PCPlus4E;
    wire [4:0] Rs1E, Rs2E, RdE;

    // Wires for Execute Stage
    wire [31:0] ALUOutM, WriteDataM;
    wire [4:0] WriteRegM;

    // Wires for Memory Stage
    wire [31:0] ReadDataW, ALUOutW, ResultW;
    wire [4:0] WriteRegW;

    FetchStage fetchStage(
        .clk(clk),
        .reset(reset),
        .PCBranchM(PCBranchM),
        .PCSrcM(PCSrcM),
        .PCF(PCF),
        .InstrF(InstrF)
    );

    DecodeStage decodeStage(
        .clk(clk),
        .reset(reset),
        .InstrD(InstrD),
        .PCPlus4D(PCPlus4D),
        .SrcAE(SrcAE),
        .SrcBE(SrcBE),
        .SignImmE(SignImmE),
        .Rs1E(Rs1E),
        .Rs2E(Rs2E),
        .RdE(RdE),
        .PCPlus4E(PCPlus4E)
    );

    ExecuteStage executeStage(
        .clk(clk),
        .reset(reset),
        .SrcAE(SrcAE),
        .SrcBE(SrcBE),
        .SignImmE(SignImmE),
        .RdE(RdE),
        .ALUOutM(ALUOutM),
        .WriteDataM(WriteDataM),
        .WriteRegM(WriteRegM)
    );

    MemoryStage memoryStage(
        .clk(clk),
        .reset(reset),
        .ALUOutM(ALUOutM),
        .WriteDataM(WriteDataM),
        .WriteRegM(WriteRegM),
        .ReadDataW(ReadDataW),
        .ALUOutW(ALUOutW),
        .WriteRegW(WriteRegW)
    );

    WriteBackStage writeBackStage(
        .clk(clk),
        .reset(reset),
        .ReadDataW(ReadDataW),
        .ALUOutW(ALUOutW),
        .WriteRegW(WriteRegW),
        .ResultW(ResultW)
    );
endmodule
//here are the submodules 
module FetchStage (
    input clk,
    input reset,
    input [31:0] PCBranchM,
    input PCSrcM,
    output reg [31:0] PCF,
    output [31:0] InstrF
);
    reg [31:0] PC;
    wire [31:0] PCPlus4F;
    reg [31:0] instruction_memory [0:255];

    assign PCPlus4F = PC + 4;
    assign InstrF = instruction_memory[PC >> 2];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC <= 32'b0;
        end else begin
            PC <= PCSrcM ? PCBranchM : PCPlus4F;
        end
        PCF <= PC;
    end
endmodule

module DecodeStage (
    input clk,
    input reset,
    input [31:0] InstrD,
    input [31:0] PCPlus4D,
    output reg [31:0] SrcAE,
    output reg [31:0] SrcBE,
    output reg [31:0] SignImmE,
    output reg [4:0] Rs1E, Rs2E, RdE,
    output reg [31:0] PCPlus4E
);
    reg [31:0] register_file [0:31];
    wire [31:0] SignImmD;

    assign SignImmD = {{20{InstrD[31]}}, InstrD[31:20]};

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            SrcAE <= 32'b0;
            SrcBE <= 32'b0;
            SignImmE <= 32'b0;
            Rs1E <= 5'b0;
            Rs2E <= 5'b0;
            RdE <= 5'b0;
            PCPlus4E <= 32'b0;
        end else begin
            SrcAE <= register_file[InstrD[19:15]];
            SrcBE <= register_file[InstrD[24:20]];
            SignImmE <= SignImmD;
            Rs1E <= InstrD[19:15];
            Rs2E <= InstrD[24:20];
            RdE <= InstrD[11:7];
            PCPlus4E <= PCPlus4D;
        end
    end
endmodule

module ExecuteStage (
    input clk,
    input reset,
    input [31:0] SrcAE,
    input [31:0] SrcBE,
    input [31:0] SignImmE,
    input [4:0] RdE,
    output reg [31:0] ALUOutM,
    output reg [31:0] WriteDataM,
    output reg [4:0] WriteRegM
);
    wire [31:0] ALUResult;
    reg [31:0] ALU;

    always @(*) begin
        case (SignImmE[5:0])
            6'b011000: ALU = SrcAE + SrcBE;  // ADD
            6'b011001: ALU = SrcAE - SrcBE;  // SUB
            6'b011010: ALU = SrcAE & SrcBE;  // AND
            6'b011011: ALU = SrcAE | SrcBE;  // OR
            6'b011100: ALU = SrcAE ^ SrcBE;  // XOR
            6'b011101: ALU = SrcAE << SrcBE; // SLL
            6'b011110: ALU = SrcAE >> SrcBE; // SRL
            6'b011111: ALU = (SrcAE < SrcBE) ? 1 : 0; // SLT
            default: ALU = 32'b0;
        endcase
    end

    assign ALUResult = ALU;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ALUOutM <= 32'b0;
            WriteDataM <= 32'b0;
            WriteRegM <= 5'b0;
        end else begin
            ALUOutM <= ALUResult;
            WriteDataM <= SrcBE;
            WriteRegM <= RdE;
        end
    end
endmodule

module MemoryStage (
    input clk,
    input reset,
    input [31:0] ALUOutM,
    input [31:0] WriteDataM,
    input [4:0] WriteRegM,
    output reg [31:0] ReadDataW,
    output reg [31:0] ALUOutW,
    output reg [4:0] WriteRegW
);
    reg [31:0] data_memory [0:255];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ReadDataW <= 32'b0;
            ALUOutW <= 32'b0;
            WriteRegW <= 5'b0;
        end else begin
            ReadDataW <= data_memory[ALUOutM >> 2];
            ALUOutW <= ALUOutM;
            WriteRegW <= WriteRegM;
        end
    end

    always @(posedge clk) begin
        data_memory[ALUOutM >> 2] <= WriteDataM;
    end
endmodule

module WriteBackStage (
    input clk,
    input reset,
    input [31:0] ReadDataW,
    input [31:0] ALUOutW,
    input [4:0] WriteRegW,
    output reg [31:0] ResultW
);
    reg [31:0] register_file [0:31];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ResultW <= 32'b0;
        end else begin
            ResultW <= WriteRegW ? (ReadDataW ? ReadDataW : ALUOutW) : 32'b0;
            register_file[WriteRegW] <= ResultW;
        end
    end
endmodule
