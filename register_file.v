// ============================================================
// register_file.v
// Holds the 4 operand registers used by the 2-wide SIMD lanes.
// Written one at a time during the ID stage (serialized because
// Tiny Tapeout only provides an 8-bit input bus).
// ============================================================
module register_file (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [1:0] write_sel,   // 0=opA_l0 1=opA_l1 2=opB_l0 3=opB_l1
    input  wire       write_en,
    input  wire [7:0] write_data,
    output reg  [7:0] opA_l0, opA_l1,
    output reg  [7:0] opB_l0, opB_l1
);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            opA_l0 <= 8'h00; opA_l1 <= 8'h00;
            opB_l0 <= 8'h00; opB_l1 <= 8'h00;
        end else if (write_en) begin
            case (write_sel)
                2'd0: opA_l0 <= write_data;
                2'd1: opA_l1 <= write_data;
                2'd2: opB_l0 <= write_data;
                2'd3: opB_l1 <= write_data;
                default: ;
            endcase
        end
    end
endmodule
