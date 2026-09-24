// ============================================================
// gpu_exec_unit.v -- top level
// Pin mapping:
//   ui_in       : opcode (IF), operands (ID), LOAD data (MEM)
//   uo_out      : writeback result (WB) / STORE data (MEM)
//   uio_in[0]   : valid_in     uio_in[1]  : mem_ready
//   uio_out[2]  : busy_out     uio_out[3] : done_out
//   uio_out[7:4]: addr_out[3:0] (4-bit address, pin-budget limited)
// ============================================================
`timescale 1ns/1ps

module gpu_exec_unit (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe
);
    localparam OP_LOAD = 3'd6;

    wire valid_in  = uio_in[0];
    wire mem_ready = uio_in[1];
    wire halt_in   = 1'b0; // no external halt source in this design

    wire        busy_out, done_out;
    wire [7:0]  opcode_out;
    wire [1:0]  reg_write_sel;
    wire        reg_write_en;
    wire        en_add, en_sub, en_shift, en_relu;
    wire        en_mac_mult, en_mac_acc, en_mac_sat;
    wire        en_latch_addr;
    wire        en_capture_load0, en_capture_load1;
    wire        en_capture_store0, en_capture_store1;
    wire        wb_lane_sel, wb_start;

    wire [7:0]  opA_l0, opA_l1, opB_l0, opB_l1;
    wire [7:0]  result_l0, result_l1;
    wire [7:0]  addr_reg_out, load_data_l0, load_data_l1, store_data_out;

    controller_fsm u_ctrl (
        .clk(clk), .rst_n(rst_n),
        .valid_in(valid_in), .opcode_in(ui_in),
        .mem_ready(mem_ready), .halt_in(halt_in),
        .busy_out(busy_out), .done_out(done_out), .opcode_out(opcode_out),
        .reg_write_sel(reg_write_sel), .reg_write_en(reg_write_en),
        .en_add(en_add), .en_sub(en_sub), .en_shift(en_shift), .en_relu(en_relu),
        .en_mac_mult(en_mac_mult), .en_mac_acc(en_mac_acc), .en_mac_sat(en_mac_sat),
        .en_latch_addr(en_latch_addr),
        .en_capture_load0(en_capture_load0), .en_capture_load1(en_capture_load1),
        .en_capture_store0(en_capture_store0), .en_capture_store1(en_capture_store1),
        .wb_lane_sel(wb_lane_sel), .wb_start(wb_start)
    );

    register_file u_regs (
        .clk(clk), .rst_n(rst_n),
        .write_sel(reg_write_sel), .write_en(reg_write_en), .write_data(ui_in),
        .opA_l0(opA_l0), .opA_l1(opA_l1), .opB_l0(opB_l0), .opB_l1(opB_l1)
    );

    execute_datapath u_exec (
        .clk(clk), .rst_n(rst_n),
        .opA_l0(opA_l0), .opA_l1(opA_l1), .opB_l0(opB_l0), .opB_l1(opB_l1),
        .en_add(en_add), .en_sub(en_sub), .en_shift(en_shift), .en_relu(en_relu),
        .en_mac_mult(en_mac_mult), .en_mac_acc(en_mac_acc), .en_mac_sat(en_mac_sat),
        .result_l0(result_l0), .result_l1(result_l1)
    );

    memory_interface u_mem (
        .clk(clk), .rst_n(rst_n),
        .en_latch_addr(en_latch_addr), .addr_in(opA_l0),
        .en_capture_load0(en_capture_load0), .en_capture_load1(en_capture_load1),
        .load_bus_in(ui_in),
        .en_capture_store0(en_capture_store0), .en_capture_store1(en_capture_store1),
        .store_data_l0_in(opB_l0), .store_data_l1_in(opB_l1),
        .addr_out(addr_reg_out),
        .load_data_l0(load_data_l0), .load_data_l1(load_data_l1),
        .store_data_out(store_data_out)
    );

    // writeback mux: LOAD forwards captured memory data,
    // every other opcode forwards the ALU/MAC result
    wire       is_load   = (opcode_out[2:0] == OP_LOAD);
    wire [7:0] wb_val_l0 = is_load ? load_data_l0 : result_l0;
    wire [7:0] wb_val_l1 = is_load ? load_data_l1 : result_l1;

    reg [7:0] data_out_reg;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) data_out_reg <= 8'h00;
        else if (wb_start) data_out_reg <= wb_lane_sel ? wb_val_l1 : wb_val_l0;

    // STORE drives its data directly onto uo_out during the MEM_STORE_Lx states
    wire mem_driving_output = en_capture_store0 || en_capture_store1;
    assign uo_out = mem_driving_output ? store_data_out : data_out_reg;

    assign uio_out = {addr_reg_out[3:0], done_out, busy_out, 2'b00};
    assign uio_oe  = 8'b1111_1100; // bits[7:2] driven out, bits[1:0] are inputs
endmodule
