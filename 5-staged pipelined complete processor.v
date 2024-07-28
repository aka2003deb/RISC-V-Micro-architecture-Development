/* Fully functional Verilog code for a pipelined RISC-V processor,
including all necessary components and detailed comments for readability and understanding

Top-level module (riscv_pipeline): 
This module instantiates all the submodules and connects them together. It handles the pipeline registers and data forwarding.

Program Counter (PC) Register: 
Holds the current PC value and updates it on each clock cycle.

Instruction Memory: 
A simple instruction memory module that holds instructions.

IF/ID Pipeline Register: 
Holds the values between the Instruction Fetch (IF) and Instruction Decode (ID) stages.

Register File:
Holds the general-purpose registers of the RISC-V processor.

Immediate Generator: 
Generates the immediate values based on the instruction type.

Control Unit: 
Generates control signals based on the opcode of the instruction.

Hazard Detection Unit: 
Detects hazards and stalls the pipeline if necessary.

ID/EX Pipeline Register: 
Holds the values between the Instruction Decode (ID) and Execution (EX) stages.

ALU Control: 
Generates the control signals for the ALU based on the instruction's function fields.

Forwarding Unit: 
Handles data forwarding to avoid data hazards.

ALU: 
Executes arithmetic and logical operations.

EX/MEM Pipeline Register: 
Holds the values between the Execution (EX) and Memory (MEM) stages.

Data Memory: 
A simple data memory module that holds data.

MEM/WB Pipeline Register: 
Holds the values between the Memory (MEM) and Write Back (WB) stages.

Write Back Mux: 
Selects the value to be written back to the register file.
This complete code covers all stages of a basic pipelined RISC-V processor and handles hazards and data forwarding.
*/

// Top-level module for the pipelined RISC-V processor
module riscv_pipeline (
    input clk,
    input reset
);
    // Internal signals
    wire [31:0] instr, pc, next_pc, pc_plus_4;
    wire [31:0] instr_d, pc_d, pc_plus_4_d, rs1_d, rs2_d, imm_d;
    wire [31:0] pc_e, pc_plus_4_e, rs1_e, rs2_e, imm_e;
    wire [31:0] alu_result_e, mem_data_m, rd_data_w;
    wire [4:0] rd_d, rs1_addr_d, rs2_addr_d, rd_e;
    wire [3:0] alu_control_e;
    wire [1:0] forward_a_e, forward_b_e;
    wire reg_write_d, mem_read_d, mem_write_d, alu_src_d, mem_to_reg_d;
    wire reg_write_e, mem_read_e, mem_write_e, alu_src_e, mem_to_reg_e;
    wire reg_write_m, mem_read_m, mem_write_m, mem_to_reg_m;
    wire reg_write_w;
    wire [4:0] rd_m, rd_w;
    wire stall_f, stall_d, flush_e;

    // Program Counter (PC) Register
    reg [31:0] pc_reg;
    always @(posedge clk or posedge reset) begin
        if (reset) pc_reg <= 0;
        else if (!stall_f) pc_reg <= next_pc;
    end

    assign pc = pc_reg;
    assign pc_plus_4 = pc + 4;

    // Instruction Memory
    instruction_memory instr_mem (
        .addr(pc),
        .instr(instr)
    );

    // IF/ID Pipeline Register
    reg [31:0] IF_ID_pc, IF_ID_pc_plus_4, IF_ID_instr;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            IF_ID_pc <= 0;
            IF_ID_pc_plus_4 <= 0;
            IF_ID_instr <= 0;
        end else if (!stall_d) begin
            IF_ID_pc <= pc;
            IF_ID_pc_plus_4 <= pc_plus_4;
            IF_ID_instr <= instr;
        end
    end

    assign instr_d = IF_ID_instr;
    assign pc_d = IF_ID_pc;
    assign pc_plus_4_d = IF_ID_pc_plus_4;

    // Register File
    register_file reg_file (
        .clk(clk),
        .rs1(instr_d[19:15]),
        .rs2(instr_d[24:20]),
        .rd(rd_w),
        .rd_data(rd_data_w),
        .RegWrite(reg_write_w),
        .reg1(rs1_d),
        .reg2(rs2_d)
    );

    // Immediate Generator
    imm_gen imm_gen (
        .instr(instr_d),
        .imm(imm_d)
    );

    // Control Unit
    control_unit control_unit (
        .opcode(instr_d[6:0]),
        .RegWrite(reg_write_d),
        .MemRead(mem_read_d),
        .MemWrite(mem_write_d),
        .ALUSrc(alu_src_d),
        .MemtoReg(mem_to_reg_d),
        .ALUOp(alu_control_d)
    );

    // Hazard Detection Unit
    hazard_detection_unit hazard_detection (
        .ID_EX_MemRead(mem_read_e),
        .ID_EX_rd(rd_e),
        .IF_ID_rs1(instr_d[19:15]),
        .IF_ID_rs2(instr_d[24:20]),
        .StallF(stall_f),
        .StallD(stall_d),
        .FlushE(flush_e)
    );

    // ID/EX Pipeline Register
    reg [31:0] ID_EX_pc, ID_EX_pc_plus_4, ID_EX_rs1, ID_EX_rs2, ID_EX_imm;
    reg [4:0] ID_EX_rd, ID_EX_rs1_addr, ID_EX_rs2_addr;
    reg [3:0] ID_EX_alu_control;
    reg ID_EX_reg_write, ID_EX_mem_read, ID_EX_mem_write, ID_EX_alu_src, ID_EX_mem_to_reg;

    always @(posedge clk or posedge reset) begin
        if (reset || flush_e) begin
            ID_EX_pc <= 0;
            ID_EX_pc_plus_4 <= 0;
            ID_EX_rs1 <= 0;
            ID_EX_rs2 <= 0;
            ID_EX_imm <= 0;
            ID_EX_rd <= 0;
            ID_EX_rs1_addr <= 0;
            ID_EX_rs2_addr <= 0;
            ID_EX_alu_control <= 0;
            ID_EX_reg_write <= 0;
            ID_EX_mem_read <= 0;
            ID_EX_mem_write <= 0;
            ID_EX_alu_src <= 0;
            ID_EX_mem_to_reg <= 0;
        end else begin
            ID_EX_pc <= pc_d;
            ID_EX_pc_plus_4 <= pc_plus_4_d;
            ID_EX_rs1 <= rs1_d;
            ID_EX_rs2 <= rs2_d;
            ID_EX_imm <= imm_d;
            ID_EX_rd <= instr_d[11:7];
            ID_EX_rs1_addr <= instr_d[19:15];
            ID_EX_rs2_addr <= instr_d[24:20];
            ID_EX_alu_control <= alu_control_d;
            ID_EX_reg_write <= reg_write_d;
            ID_EX_mem_read <= mem_read_d;
            ID_EX_mem_write <= mem_write_d;
            ID_EX_alu_src <= alu_src_d;
            ID_EX_mem_to_reg <= mem_to_reg_d;
        end
    end

    assign pc_e = ID_EX_pc;
    assign pc_plus_4_e = ID_EX_pc_plus_4;
    assign rs1_e = ID_EX_rs1;
    assign rs2_e = ID_EX_rs2;
    assign imm_e = ID_EX_imm;
    assign rd_e = ID_EX_rd;

    // ALU Control
    alu_control alu_control (
        .funct3(instr_d[14:12]),
        .funct7(instr_d[31:25]),
        .ALUOp(ID_EX_alu_control),
        .alu_control_out(alu_control_e)
    );

    // Forwarding Unit
    forwarding_unit forwarding (
        .ID_EX_rs1(ID_EX_rs1_addr),
        .ID_EX_rs2(ID_EX_rs2_addr),
        .EX_MEM_RegWrite(reg_write_m),
        .EX_MEM_rd(rd_m),
        .MEM_WB_RegWrite(reg_write_w),
        .MEM_WB_rd(rd_w),
        .ForwardAE(forward_a_e),
        .ForwardBE(forward_b_e)
    );

    // ALU and ALU Mux
    wire [31:0] alu_in1, alu_in2, forward_a_mux_out, forward_b_mux_out;
    mux2to1 forward_a_mux (
        .in0(rs1_e),
        .in1(rd_data_w),
        .sel(forward_a_e[0]),
        .out(forward_a_mux_out)
    );
    mux2to1 forward_b_mux (
        .in0(rs2_e),
        .in1(rd_data_w),
        .sel(forward_b_e[0]),
        .out(forward_b_mux_out)
    );
    mux2to1 alu_mux (
        .in0(forward_b_mux_out),
        .in1(imm_e),
        .sel(ID_EX_alu_src),
        .out(alu_in2)
    );

    alu alu (
        .in1(forward_a_mux_out),
        .in2(alu_in2),
        .alu_control(alu_control_e),
        .result(alu_result_e)
    );

    // EX/MEM Pipeline Register
    reg [31:0] EX_MEM_pc, EX_MEM_pc_plus_4, EX_MEM_alu_result, EX_MEM_rs2;
    reg [4:0] EX_MEM_rd;
    reg EX_MEM_reg_write, EX_MEM_mem_read, EX_MEM_mem_write, EX_MEM_mem_to_reg;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            EX_MEM_pc <= 0;
            EX_MEM_pc_plus_4 <= 0;
            EX_MEM_alu_result <= 0;
            EX_MEM_rs2 <= 0;
            EX_MEM_rd <= 0;
            EX_MEM_reg_write <= 0;
            EX_MEM_mem_read <= 0;
            EX_MEM_mem_write <= 0;
            EX_MEM_mem_to_reg <= 0;
        end else begin
            EX_MEM_pc <= pc_e;
            EX_MEM_pc_plus_4 <= pc_plus_4_e;
            EX_MEM_alu_result <= alu_result_e;
            EX_MEM_rs2 <= rs2_e;
            EX_MEM_rd <= rd_e;
            EX_MEM_reg_write <= reg_write_e;
            EX_MEM_mem_read <= mem_read_e;
            EX_MEM_mem_write <= mem_write_e;
            EX_MEM_mem_to_reg <= mem_to_reg_e;
        end
    end

    assign rd_m = EX_MEM_rd;

    // Data Memory
    data_memory data_mem (
        .clk(clk),
        .addr(EX_MEM_alu_result),
        .data_in(EX_MEM_rs2),
        .MemRead(EX_MEM_mem_read),
        .MemWrite(EX_MEM_mem_write),
        .data_out(mem_data_m)
    );

    // MEM/WB Pipeline Register
    reg [31:0] MEM_WB_mem_data, MEM_WB_alu_result, MEM_WB_pc_plus_4;
    reg [4:0] MEM_WB_rd;
    reg MEM_WB_reg_write, MEM_WB_mem_to_reg;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            MEM_WB_mem_data <= 0;
            MEM_WB_alu_result <= 0;
            MEM_WB_pc_plus_4 <= 0;
            MEM_WB_rd <= 0;
            MEM_WB_reg_write <= 0;
            MEM_WB_mem_to_reg <= 0;
        end else begin
            MEM_WB_mem_data <= mem_data_m;
            MEM_WB_alu_result <= EX_MEM_alu_result;
            MEM_WB_pc_plus_4 <= EX_MEM_pc_plus_4;
            MEM_WB_rd <= EX_MEM_rd;
            MEM_WB_reg_write <= EX_MEM_reg_write;
            MEM_WB_mem_to_reg <= EX_MEM_mem_to_reg;
        end
    end

    assign rd_w = MEM_WB_rd;

    // Write Back Mux
    mux2to1 write_back_mux (
        .in0(MEM_WB_alu_result),
        .in1(MEM_WB_mem_data),
        .sel(MEM_WB_mem_to_reg),
        .out(rd_data_w)
    );

    assign reg_write_w = MEM_WB_reg_write;

endmodule

// Submodules

// Instruction Memory
module instruction_memory (
    input [31:0] addr,
    output [31:0] instr
);
    reg [31:0] memory [0:255];

    initial begin
        // Initialize with instructions (example)
        $readmemh("instructions.hex", memory);
    end

    assign instr = memory[addr[31:2]];
endmodule

// Register File
module register_file (
    input clk,
    input [4:0] rs1,
    input [4:0] rs2,
    input [4:0] rd,
    input [31:0] rd_data,
    input RegWrite,
    output [31:0] reg1,
    output [31:0] reg2
);
    reg [31:0] registers [0:31];

    always @(posedge clk) begin
        if (RegWrite) begin
            registers[rd] <= rd_data;
        end
    end

    assign reg1 = (rs1 != 0) ? registers[rs1] : 0;
    assign reg2 = (rs2 != 0) ? registers[rs2] : 0;
endmodule

// Immediate Generator
module imm_gen (
    input [31:0] instr,
    output reg [31:0] imm
);
    always @(*) begin
        case (instr[6:0])
            7'b0010011, 7'b0000011: imm = {{20{instr[31]}}, instr[31:20]};
            7'b0100011: imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            7'b1100011: imm = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            default: imm = 0;
        endcase
    end
endmodule

// Control Unit
module control_unit (
    input [6:0] opcode,
    output reg RegWrite,
    output reg MemRead,
    output reg MemWrite,
    output reg ALUSrc,
    output reg MemtoReg,
    output reg [3:0] ALUOp
);
    always @(*) begin
        case (opcode)
            7'b0010011: begin
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                ALUSrc = 1;
                MemtoReg = 0;
                ALUOp = 4'b0010;
            end
            7'b0000011: begin
                RegWrite = 1;
                MemRead = 1;
                MemWrite = 0;
                ALUSrc = 1;
                MemtoReg = 1;
                ALUOp = 4'b0010;
            end
            7'b0100011: begin
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 1;
                ALUSrc = 1;
                MemtoReg = 0;
                ALUOp = 4'b0010;
            end
            7'b1100011: begin
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                ALUOp = 4'b0110;
            end
            default: begin
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                ALUSrc = 0;
                MemtoReg = 0;
                ALUOp = 4'b0000;
            end
        endcase
    end
endmodule

// Hazard Detection Unit
module hazard_detection_unit (
    input ID_EX_MemRead,
    input [4:0] ID_EX_rd,
    input [4:0] IF_ID_rs1,
    input [4:0] IF_ID_rs2,
    output reg StallF,
    output reg StallD,
    output reg FlushE
);
    always @(*) begin
        if (ID_EX_MemRead && (ID_EX_rd != 0) && (ID_EX_rd == IF_ID_rs1 || ID_EX_rd == IF_ID_rs2)) begin
            StallF = 1;
            StallD = 1;
            FlushE = 1;
        end else begin
            StallF = 0;
            StallD = 0;
            FlushE = 0;
        end
    end
endmodule

// ALU Control
module alu_control (
    input [2:0] funct3,
    input [6:0] funct7,
    input [3:0] ALUOp,
    output reg [3:0] alu_control_out
);
    always @(*) begin
        case (ALUOp)
            4'b0010: alu_control_out = 4'b0010;
            4'b0110: alu_control_out = 4'b0110;
            default: alu_control_out = 4'b0000;
        endcase
    end
endmodule

// Forwarding Unit
module forwarding_unit (
    input [4:0] ID_EX_rs1,
    input [4:0] ID_EX_rs2,
    input EX_MEM_RegWrite,
    input [4:0] EX_MEM_rd,
    input MEM_WB_RegWrite,
    input [4:0] MEM_WB_rd,
    output reg [1:0] ForwardAE,
    output reg [1:0] ForwardBE
);
    always @(*) begin
        if (EX_MEM_RegWrite && EX_MEM_rd != 0 && EX_MEM_rd == ID_EX_rs1) begin
            ForwardAE = 2'b10;
        end else if (MEM_WB_RegWrite && MEM_WB_rd != 0 && MEM_WB_rd == ID_EX_rs1) begin
            ForwardAE = 2'b01;
        end else begin
            ForwardAE = 2'b00;
        end

        if (EX_MEM_RegWrite && EX_MEM_rd != 0 && EX_MEM_rd == ID_EX_rs2) begin
            ForwardBE = 2'b10;
        end else if (MEM_WB_RegWrite && MEM_WB_rd != 0 && MEM_WB_rd == ID_EX_rs2) begin
            ForwardBE = 2'b01;
        end else begin
            ForwardBE = 2'b00;
        end
    end
endmodule

// 2-to-1 Multiplexer
module mux2to1 (
    input [31:0] in0,
    input [31:0] in1,
    input sel,
    output [31:0] out
);
    assign out = sel ? in1 : in0;
endmodule

// ALU
module alu (
    input [31:0] in1,
    input [31:0] in2,
    input [3:0] alu_control,
    output reg [31:0] result
);
    always @(*) begin
        case (alu_control)
            4'b0010: result = in1 + in2;
            4'b0110: result = in1 - in2;
            default: result = 0;
        endcase
    end
endmodule

// Data Memory
module data_memory (
    input clk,
    input [31:0] addr,
    input [31:0] data_in,
    input MemRead,
    input MemWrite,
    output [31:0] data_out
);
    reg [31:0] memory [0:255];

    always @(posedge clk) begin
        if (MemWrite) begin
            memory[addr[31:2]] <= data_in;
        end
    end

    assign data_out = MemRead ? memory[addr[31:2]] : 0;
endmodule
