`timescale 1ns/1ps

module reduce_sigmoid #(
    parameter int N_PE = 6,
    parameter ROM_FILE = "sigmoid_lut.mem"
) (
    input logic clk_i,
    input logic rst_n_i,

    input rs_types_pkg::seq_to_rs_t seq_to_rs_data_i,
    input  logic seq_to_rs_start_i,
    output logic seq_to_rs_ready_o,

    input rs_types_pkg::pe_to_rs_t
        pe_to_rs_data_i [0:N_PE-1],

    input  logic [N_PE-1:0] pe_to_rs_valid_i,
    output logic [N_PE-1:0] pe_to_rs_ready_o,

    output rs_types_pkg::rs_to_seq_t rs_to_seq_data_o,
    output logic rs_to_seq_valid_o,
    input  logic rs_to_seq_ready_i
);

    rs_types_pkg::rs_to_seq_t reduce_data;

    logic reduce_ready;
    logic reduce_valid;
    logic sigmoid_ready;

    logic busy;

    // Новый запрос разрешается после приёма
    // результата предыдущего нейрона.
    assign seq_to_rs_ready_o = !busy && reduce_ready;

    reduce #(
        .N_PE(N_PE)
    ) u_reduce (
        .clk_i  (clk_i),
        .rst_n_i(rst_n_i),

        .seq_to_reduce_data_i(seq_to_rs_data_i),
        .seq_to_reduce_start_i(
            seq_to_rs_start_i && !busy
        ),
        .seq_to_reduce_ready_o(reduce_ready),

        .pe_to_reduce_data_i (pe_to_rs_data_i),
        .pe_to_reduce_valid_i(pe_to_rs_valid_i),
        .pe_to_reduce_ready_o(pe_to_rs_ready_o),

        .reduce_to_sigmoid_data_o (reduce_data),
        .reduce_to_sigmoid_valid_o(reduce_valid),
        .reduce_to_sigmoid_ready_i(sigmoid_ready)
    );

    sigmoid #(
        .ROM_FILE(ROM_FILE)
    ) u_sigmoid (
        .clk_i  (clk_i),
        .rst_n_i(rst_n_i),

        .reduce_to_sigmoid_data_i (reduce_data),
        .reduce_to_sigmoid_valid_i(reduce_valid),
        .reduce_to_sigmoid_ready_o(sigmoid_ready),

        .sigmoid_to_rs_data_o (rs_to_seq_data_o),
        .sigmoid_to_rs_valid_o(rs_to_seq_valid_o),
        .sigmoid_to_rs_ready_i(rs_to_seq_ready_i)
    );

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            busy <= 0;

        end else if (
            seq_to_rs_start_i &&
            seq_to_rs_ready_o
        ) begin
            busy <= 1;

        end else if (
            rs_to_seq_valid_o &&
            rs_to_seq_ready_i
        ) begin
            busy <= 0;
        end
    end

endmodule