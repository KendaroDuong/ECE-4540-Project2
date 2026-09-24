// ============================================================
// execute_datapath.v
// 2-lane ALU + MAC datapath. Each enable is pulsed by
// controller_fsm for exactly one cycle per EX state.
// ============================================================
module execute_datapath (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] opA_l0, opA_l1,
    input  wire [7:0] opB_l0, opB_l1,
    input  wire       en_add, en_sub, en_shift, en_relu,
    input  wire       en_mac_mult, en_mac_acc, en_mac_sat,
    output reg  [7:0] result_l0, result_l1
);
    reg [15:0] prod_l0, prod_l1;
    reg [15:0] acc_l0, acc_l1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            result_l0 <= 8'h00; result_l1 <= 8'h00;
            prod_l0   <= 16'h0; prod_l1   <= 16'h0;
            acc_l0    <= 16'h0; acc_l1    <= 16'h0;
        end else begin
            if (en_add) begin
                result_l0 <= opA_l0 + opB_l0;
                result_l1 <= opA_l1 + opB_l1;
            end
            if (en_sub) begin
                result_l0 <= opA_l0 - opB_l0;
                result_l1 <= opA_l1 - opB_l1;
            end
            if (en_shift) begin
                result_l0 <= opA_l0 << opB_l0[2:0];
                result_l1 <= opA_l1 << opB_l1[2:0];
            end
            if (en_relu) begin
                result_l0 <= opA_l0[7] ? 8'h00 : opA_l0;
                result_l1 <= opA_l1[7] ? 8'h00 : opA_l1;
            end
            if (en_mac_mult) begin
                prod_l0 <= opA_l0 * opB_l0;
                prod_l1 <= opA_l1 * opB_l1;
            end
            if (en_mac_acc) begin
                acc_l0 <= acc_l0 + prod_l0;
                acc_l1 <= acc_l1 + prod_l1;
            end
            if (en_mac_sat) begin
                result_l0 <= (acc_l0 > 16'd255) ? 8'hFF : acc_l0[7:0];
                result_l1 <= (acc_l1 > 16'd255) ? 8'hFF : acc_l1[7:0];
            end
        end
    end
endmodule
