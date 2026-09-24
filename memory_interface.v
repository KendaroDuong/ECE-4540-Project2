// ============================================================
// memory_interface.v
// Handles LOAD/STORE address and data movement over the shared
// bus. Address supplied is opA_l0; STORE data source is opB_l0/l1.
// ============================================================
`timescale 1ns/1ps

module memory_interface (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       en_latch_addr,
    input  wire [7:0] addr_in,
    input  wire       en_capture_load0, en_capture_load1,
    input  wire [7:0] load_bus_in,
    input  wire       en_capture_store0, en_capture_store1,
    input  wire [7:0] store_data_l0_in, store_data_l1_in,
    output reg  [7:0] addr_out,
    output reg  [7:0] load_data_l0, load_data_l1,
    output wire [7:0] store_data_out
);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            addr_out     <= 8'h00;
            load_data_l0 <= 8'h00;
            load_data_l1 <= 8'h00;
        end else begin
            if (en_latch_addr)    addr_out     <= addr_in;
            if (en_capture_load0) load_data_l0 <= load_bus_in;
            if (en_capture_load1) load_data_l1 <= load_bus_in;
        end
    end

    // combinational so it's valid the same cycle the capture enable fires
    assign store_data_out = en_capture_store1 ? store_data_l1_in : store_data_l0_in;
endmodule
