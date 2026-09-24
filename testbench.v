// ============================================================
// testbench.v
// Drives gpu_exec_unit through one instruction of every opcode
// and checks the writeback result against a hand-computed
// expected value. Prints PASS/FAIL per test and a final summary.
//
// Simplification: mem_ready is tied high (no real memory model /
// no address checking) -- LOAD data is supplied directly by the
// testbench via cur_load0/cur_load1.
// ============================================================
`timescale 1ns/1ps

module testbench;
    reg        clk;
    reg        rst_n;
    reg  [7:0] ui_in;
    wire [7:0] uo_out;
    wire [7:0] uio_in;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    reg valid_in;
    reg mem_ready;
    assign uio_in = {6'b0, mem_ready, valid_in};

    localparam OP_NOP=3'd0, OP_ADD=3'd1, OP_SUB=3'd2, OP_SHIFT=3'd3,
               OP_RELU=3'd4, OP_MAC=3'd5, OP_LOAD=3'd6, OP_STORE=3'd7;

    gpu_exec_unit dut (
        .clk(clk), .rst_n(rst_n),
        .ui_in(ui_in), .uo_out(uo_out),
        .uio_in(uio_in), .uio_out(uio_out), .uio_oe(uio_oe)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    // ---- stimulus values for the instruction currently in flight ----
    reg [7:0] cur_opcode, cur_a0, cur_a1, cur_b0, cur_b1;
    reg [7:0] cur_load0, cur_load1;

    // Drive ui_in based on the DUT's controller state. Sampled on
    // negedge so the state read is race-free relative to the
    // posedge-clocked state register inside controller_fsm.
    always @(negedge clk) begin
        case (dut.u_ctrl.state)
            dut.u_ctrl.IF_FETCH_OPCODE: ui_in <= cur_opcode;
            dut.u_ctrl.ID_FETCH_A0:     ui_in <= cur_a0;
            dut.u_ctrl.ID_FETCH_A1:     ui_in <= cur_a1;
            dut.u_ctrl.ID_FETCH_B0:     ui_in <= cur_b0;
            dut.u_ctrl.ID_FETCH_B1:     ui_in <= cur_b1;
            dut.u_ctrl.MEM_LOAD_L0:     ui_in <= cur_load0;
            dut.u_ctrl.MEM_LOAD_L1:     ui_in <= cur_load1;
            default:                    ui_in <= 8'h00;
        endcase
    end

    integer errors = 0;

    task automatic wait_for_idle;
        begin
            wait (dut.u_ctrl.state == dut.u_ctrl.IF_IDLE);
            @(negedge clk);
        end
    endtask

    task automatic send_instruction;
        input [7:0] opcode, a0, a1, b0, b1;
        begin
            wait_for_idle;
            cur_opcode = opcode;
            cur_a0 = a0; cur_a1 = a1; cur_b0 = b0; cur_b1 = b1;
            valid_in = 1'b1;
            @(negedge clk);
            valid_in = 1'b0;
        end
    endtask

    task automatic check_result;
        input [7:0] exp_l0, exp_l1;
        input [8*8-1:0] name;
        begin
            // wb_val_l0/l1 are combinational muxes inside the DUT that are
            // already stable by EX_DONE / MEM_LOAD_L1, well before WB_DONE,
            // so reading them directly avoids any writeback-pipeline timing.
            if (dut.wb_val_l0 !== exp_l0) begin
                $display("FAIL (%0s): lane0 expected %0d got %0d", name, exp_l0, dut.wb_val_l0);
                errors = errors + 1;
            end else begin
                $display("PASS (%0s): lane0 = %0d", name, dut.wb_val_l0);
            end
            if (dut.wb_val_l1 !== exp_l1) begin
                $display("FAIL (%0s): lane1 expected %0d got %0d", name, exp_l1, dut.wb_val_l1);
                errors = errors + 1;
            end else begin
                $display("PASS (%0s): lane1 = %0d", name, dut.wb_val_l1);
            end
        end
    endtask

    initial begin
        $dumpfile("gpu_exec_unit.vcd");
        $dumpvars(0, testbench);

        rst_n = 0; valid_in = 0; mem_ready = 1; ui_in = 8'h00;
        cur_opcode = 0; cur_a0 = 0; cur_a1 = 0; cur_b0 = 0; cur_b1 = 0;
        cur_load0 = 0; cur_load1 = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(negedge clk);

        // ---- ADD: 10+5=15, 20+7=27 ----
        send_instruction(OP_ADD, 8'd10, 8'd20, 8'd5, 8'd7);
        wait (dut.u_ctrl.state == dut.u_ctrl.WB_DONE);
        check_result(8'd15, 8'd27, "ADD");

        // ---- SUB: 50-20=30, 60-15=45 ----
        send_instruction(OP_SUB, 8'd50, 8'd60, 8'd20, 8'd15);
        wait (dut.u_ctrl.state == dut.u_ctrl.WB_DONE);
        check_result(8'd30, 8'd45, "SUB");

        // ---- SHIFT: 1<<3=8, 1<<2=4 ----
        send_instruction(OP_SHIFT, 8'd1, 8'd1, 8'd3, 8'd2);
        wait (dut.u_ctrl.state == dut.u_ctrl.WB_DONE);
        check_result(8'd8, 8'd4, "SHIFT");

        // ---- RELU: negative -> 0, positive -> unchanged ----
        send_instruction(OP_RELU, 8'hF0, 8'd25, 8'h00, 8'h00);
        wait (dut.u_ctrl.state == dut.u_ctrl.WB_DONE);
        check_result(8'd0, 8'd25, "RELU");

        // ---- MAC: 10*10=100, 5*5=25 (single accumulate, starts from 0) ----
        send_instruction(OP_MAC, 8'd10, 8'd5, 8'd10, 8'd5);
        wait (dut.u_ctrl.state == dut.u_ctrl.WB_DONE);
        check_result(8'd100, 8'd25, "MAC");

        // ---- LOAD: forwards testbench-supplied "memory" contents ----
        cur_load0 = 8'hAA; cur_load1 = 8'hBB;
        send_instruction(OP_LOAD, 8'd0, 8'd0, 8'd0, 8'd0);
        wait (dut.u_ctrl.state == dut.u_ctrl.WB_DONE);
        check_result(8'hAA, 8'hBB, "LOAD");

        // ---- STORE: check data appears on uo_out during MEM_STORE_L0/L1 ----
        send_instruction(OP_STORE, 8'd0, 8'd0, 8'hCC, 8'hDD);
        wait (dut.u_ctrl.state == dut.u_ctrl.MEM_STORE_L0);
        @(posedge clk);
        if (uo_out !== 8'hCC) begin
            $display("FAIL (STORE): lane0 expected %0d got %0d", 8'hCC, uo_out);
            errors = errors + 1;
        end else $display("PASS (STORE): lane0 = %0d", uo_out);
        wait (dut.u_ctrl.state == dut.u_ctrl.MEM_STORE_L1);
        @(posedge clk);
        if (uo_out !== 8'hDD) begin
            $display("FAIL (STORE): lane1 expected %0d got %0d", 8'hDD, uo_out);
            errors = errors + 1;
        end else $display("PASS (STORE): lane1 = %0d", uo_out);
        wait (dut.u_ctrl.state == dut.u_ctrl.WB_DONE);

        repeat (5) @(posedge clk);
        if (errors == 0)
            $display("ALL TESTS PASSED");
        else
            $display("%0d TEST(S) FAILED", errors);

        $finish;
    end
endmodule
