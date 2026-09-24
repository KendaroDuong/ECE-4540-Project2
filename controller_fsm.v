// ============================================================
// controller_fsm.v
// Single controller FSM organized into 5 stages: IF, ID, EX, MEM, WB
// 34 states total. One instruction moves through the stages
// sequentially (not overlapped / not a pipelined design).
// ============================================================
`timescale 1ns/1ps

module controller_fsm (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       valid_in,
    input  wire [7:0] opcode_in,     // from ui_in, sampled during IF_FETCH_OPCODE
    input  wire       mem_ready,     // from uio_in
    input  wire       halt_in,

    output reg        busy_out,
    output reg        done_out,
    output reg  [7:0] opcode_out,    // latched opcode, exposed for top-level WB muxing

    // ID stage: register file write port
    output reg  [1:0] reg_write_sel, // 0=opA_l0 1=opA_l1 2=opB_l0 3=opB_l1
    output reg        reg_write_en,

    // EX stage: datapath operation enables (one pulses per cycle)
    output reg        en_add, en_sub, en_shift, en_relu,
    output reg        en_mac_mult, en_mac_acc, en_mac_sat,

    // EX_ADDR / MEM stage: memory interface controls
    output reg        en_latch_addr,
    output reg        en_capture_load0, en_capture_load1,
    output reg        en_capture_store0, en_capture_store1,

    // WB stage
    output reg        wb_lane_sel,   // 0 = lane0, 1 = lane1
    output reg        wb_start
);
    localparam OP_NOP=3'd0, OP_ADD=3'd1, OP_SUB=3'd2, OP_SHIFT=3'd3,
               OP_RELU=3'd4, OP_MAC=3'd5, OP_LOAD=3'd6, OP_STORE=3'd7;

    // ---- 34 states across 5 stages ----
    localparam IF_RESET        = 6'd0;
    localparam IF_IDLE         = 6'd1;
    localparam IF_WAIT_VALID   = 6'd2;
    localparam IF_FETCH_OPCODE = 6'd3;
    localparam IF_BUSY         = 6'd4;

    localparam ID_DECODE       = 6'd5;
    localparam ID_FETCH_A0     = 6'd6;
    localparam ID_FETCH_A1     = 6'd7;
    localparam ID_FETCH_B0     = 6'd8;
    localparam ID_FETCH_B1     = 6'd9;
    localparam ID_CHECK        = 6'd10;
    localparam ID_DONE         = 6'd11;

    localparam EX_ADD          = 6'd12;
    localparam EX_SUB          = 6'd13;
    localparam EX_SHIFT        = 6'd14;
    localparam EX_RELU         = 6'd15;
    localparam EX_NOP          = 6'd16;
    localparam EX_MAC_MULT     = 6'd17;
    localparam EX_MAC_ACC      = 6'd18;
    localparam EX_MAC_SAT      = 6'd19;
    localparam EX_ADDR         = 6'd20;
    localparam EX_DONE         = 6'd21;

    localparam MEM_LOAD_ADDR   = 6'd22;
    localparam MEM_LOAD_WAIT   = 6'd23;
    localparam MEM_LOAD_L0     = 6'd24;
    localparam MEM_LOAD_L1     = 6'd25;
    localparam MEM_STORE_ADDR  = 6'd26;
    localparam MEM_STORE_WAIT  = 6'd27;
    localparam MEM_STORE_L0    = 6'd28;
    localparam MEM_STORE_L1    = 6'd29;

    localparam WB_LANE0        = 6'd30;
    localparam WB_LANE1        = 6'd31;
    localparam WB_DONE         = 6'd32;
    localparam WB_HALT         = 6'd33;

    reg [5:0] state, next_state;
    reg [7:0] opcode_latched;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) state <= IF_RESET;
        else        state <= next_state;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) opcode_latched <= 8'h00;
        else if (state == IF_FETCH_OPCODE) opcode_latched <= opcode_in;

    always @(*) opcode_out = opcode_latched;

    always @(*) begin
        next_state        = state;
        busy_out          = 1'b1;
        done_out          = 1'b0;
        reg_write_sel     = 2'd0;
        reg_write_en      = 1'b0;
        en_add = 1'b0; en_sub = 1'b0; en_shift = 1'b0; en_relu = 1'b0;
        en_mac_mult = 1'b0; en_mac_acc = 1'b0; en_mac_sat = 1'b0;
        en_latch_addr     = 1'b0;
        en_capture_load0  = 1'b0; en_capture_load1  = 1'b0;
        en_capture_store0 = 1'b0; en_capture_store1 = 1'b0;
        wb_lane_sel       = 1'b0;
        wb_start          = 1'b0;

        case (state)
            // ---------------- IF ----------------
            IF_RESET: begin busy_out = 1'b0; next_state = IF_IDLE; end
            IF_IDLE:  begin busy_out = 1'b0; next_state = valid_in ? IF_WAIT_VALID : IF_IDLE; end
            IF_WAIT_VALID:   next_state = IF_FETCH_OPCODE;
            IF_FETCH_OPCODE: next_state = IF_BUSY;
            IF_BUSY:         next_state = ID_DECODE;

            // ---------------- ID ----------------
            ID_DECODE:   next_state = ID_FETCH_A0;
            ID_FETCH_A0: begin reg_write_sel = 2'd0; reg_write_en = 1'b1; next_state = ID_FETCH_A1; end
            ID_FETCH_A1: begin reg_write_sel = 2'd1; reg_write_en = 1'b1; next_state = ID_FETCH_B0; end
            ID_FETCH_B0: begin reg_write_sel = 2'd2; reg_write_en = 1'b1; next_state = ID_FETCH_B1; end
            ID_FETCH_B1: begin reg_write_sel = 2'd3; reg_write_en = 1'b1; next_state = ID_CHECK;   end
            ID_CHECK:    next_state = ID_DONE;
            ID_DONE: begin
                case (opcode_latched[2:0])
                    OP_ADD:   next_state = EX_ADD;
                    OP_SUB:   next_state = EX_SUB;
                    OP_SHIFT: next_state = EX_SHIFT;
                    OP_RELU:  next_state = EX_RELU;
                    OP_MAC:   next_state = EX_MAC_MULT;
                    OP_LOAD, OP_STORE: next_state = EX_ADDR;
                    default:  next_state = EX_NOP;
                endcase
            end

            // ---------------- EX ----------------
            EX_ADD:   begin en_add   = 1'b1; next_state = EX_DONE; end
            EX_SUB:   begin en_sub   = 1'b1; next_state = EX_DONE; end
            EX_SHIFT: begin en_shift = 1'b1; next_state = EX_DONE; end
            EX_RELU:  begin en_relu  = 1'b1; next_state = EX_DONE; end
            EX_NOP:   next_state = EX_DONE;
            EX_MAC_MULT: begin en_mac_mult = 1'b1; next_state = EX_MAC_ACC; end
            EX_MAC_ACC:  begin en_mac_acc  = 1'b1; next_state = EX_MAC_SAT; end
            EX_MAC_SAT:  begin en_mac_sat  = 1'b1; next_state = EX_DONE;    end
            EX_ADDR: begin
                en_latch_addr = 1'b1;
                next_state = (opcode_latched[2:0] == OP_LOAD) ? MEM_LOAD_ADDR : MEM_STORE_ADDR;
            end
            EX_DONE: next_state = WB_LANE0;

            // ---------------- MEM ----------------
            MEM_LOAD_ADDR:  next_state = MEM_LOAD_WAIT;
            MEM_LOAD_WAIT:  next_state = mem_ready ? MEM_LOAD_L0 : MEM_LOAD_WAIT;
            MEM_LOAD_L0:    begin en_capture_load0 = 1'b1; next_state = MEM_LOAD_L1; end
            MEM_LOAD_L1:    begin en_capture_load1 = 1'b1; next_state = WB_LANE0;    end

            MEM_STORE_ADDR:  next_state = MEM_STORE_WAIT;
            MEM_STORE_WAIT:  next_state = mem_ready ? MEM_STORE_L0 : MEM_STORE_WAIT;
            MEM_STORE_L0:    begin en_capture_store0 = 1'b1; next_state = MEM_STORE_L1; end
            MEM_STORE_L1:    begin en_capture_store1 = 1'b1; next_state = WB_DONE;       end

            // ---------------- WB ----------------
            WB_LANE0: begin wb_lane_sel = 1'b0; wb_start = 1'b1; next_state = WB_LANE1; end
            WB_LANE1: begin wb_lane_sel = 1'b1; wb_start = 1'b1; next_state = WB_DONE;  end
            WB_DONE: begin
                busy_out  = 1'b0;
                done_out  = 1'b1;
                next_state = halt_in ? WB_HALT : IF_IDLE;
            end
            WB_HALT: begin
                busy_out  = 1'b1;
                next_state = halt_in ? WB_HALT : IF_IDLE;
            end

            default: next_state = IF_RESET;
        endcase
    end
endmodule
