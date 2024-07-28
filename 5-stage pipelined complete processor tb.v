`timescale 1ns / 1ps

module riscv_pipeline_tb;

    // Inputs
    reg clk;
    reg reset;

    // Outputs
    wire [31:0] pc_w;
    wire [31:0] instr_w;
    wire [31:0] alu_result_w;
    wire [31:0] mem_data_w;
    wire [31:0] rd_data_w;
    wire reg_write_w;

    // Instantiate the Unit Under Test (UUT)
    riscv_pipeline uut (
        .clk(clk),
        .reset(reset),
        .pc_w(pc_w),
        .instr_w(instr_w),
        .alu_result_w(alu_result_w),
        .mem_data_w(mem_data_w),
        .rd_data_w(rd_data_w),
        .reg_write_w(reg_write_w)
    );

    // Clock generation
    always begin
        #5 clk = ~clk;
    end

    initial begin
        // Initialize Inputs
        clk = 0;
        reset = 1;

        // Reset the processor
        #10;
        reset = 0;

        // Load instructions into instruction memory
        $readmemh("instructions.hex", uut.instruction_memory.memory);

        // Simulation time
        #200;

        // Display results
        $display("Register File:");
        $display("x1: %d", uut.register_file.registers[1]);
        $display("x2: %d", uut.register_file.registers[2]);
        $display("x3: %d", uut.register_file.registers[3]);
        $display("x4: %d", uut.register_file.registers[4]);
        $display("x5: %d", uut.register_file.registers[5]);
        $display("x6: %d", uut.register_file.registers[6]);
        $display("x7: %d", uut.register_file.registers[7]);
        $display("x8: %d", uut.register_file.registers[8]);
        $display("x9: %d", uut.register_file.registers[9]);
        $display("x10: %d", uut.register_file.registers[10]);
        $display("x11: %d", uut.register_file.registers[11]);
        // Add more register display statements as needed

        // End simulation
        $finish;
    end

endmodule
