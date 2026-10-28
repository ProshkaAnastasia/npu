`timescale 1ns/1ps

module reduce_sigmoid #(
    parameter int N_PE = 6,
    parameter ROM_FILE = "sigmoid_lut.mem"
) (
    input logic clk_i,
    input logic rst_n_i,

    // Частичные суммы одного нейрона от всех PE.
    input logic [31:0] pe_sums_i [0:N_PE-1],

    // Управление от секвенсора.
    input  logic [31:0] seq_bias_i,
    input  logic seq_start_i,
    output logic seq_start_ready_o,

    // Итоговый результат для секвенсора.
    output logic [31:0] seq_result_data_o,
    output rs_types_pkg::rs_status_t seq_result_status_o,
    output logic seq_result_valid_o,
    input  logic seq_result_ready_i
);

    logic request_active;

    logic start_accepted;
    logic result_accepted;

    logic reduction_start_ready;

    logic [31:0] reduction_result_data;
    rs_types_pkg::rs_status_t reduction_result_status;

    logic reduction_result_valid;
    logic activation_input_ready;

    assign seq_start_ready_o =
        rst_n_i &&
        !request_active &&
        reduction_start_ready;

    assign start_accepted =
        seq_start_i && seq_start_ready_o;

    assign result_accepted =
        seq_result_valid_o && seq_result_ready_i;

    reduce #(
        .N_PE(N_PE)
    ) u_reduce (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),

        .pe_sums_i(pe_sums_i),
        .bias_i(seq_bias_i),

        .start_i(start_accepted),
        .start_ready_o(reduction_start_ready),

        .result_data_o(reduction_result_data),
        .result_status_o(reduction_result_status),
        .result_valid_o(reduction_result_valid),
        .result_ready_i(activation_input_ready)
    );

    sigmoid #(
        .ROM_FILE(ROM_FILE)
    ) u_sigmoid (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),

        .input_data_i(reduction_result_data),
        .input_status_i(reduction_result_status),
        .input_valid_i(reduction_result_valid),
        .input_ready_o(activation_input_ready),

        .result_data_o(seq_result_data_o),
        .result_status_o(seq_result_status_o),
        .result_valid_o(seq_result_valid_o),
        .result_ready_i(seq_result_ready_i)
    );

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i)
            request_active <= 1'b0;
        else if (start_accepted)
            request_active <= 1'b1;
        else if (result_accepted)
            request_active <= 1'b0;
    end

endmodule