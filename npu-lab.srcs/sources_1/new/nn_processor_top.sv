`timescale 1ns/1ps

module nn_processor_top #(
    parameter int N_PE = 6,

    parameter int CLK_FREQ_HZ = 100_000_000,
    parameter int UART_BAUD_RATE = 115_200,

    parameter ROM_FILE = "sigmoid_lut.mem"
) (
    input logic clk_i,
    input logic rst_n_i,

    input  logic uart_rx_i,
    output logic uart_tx_o,

    output logic model_valid_o,
    output logic busy_o
);

    import io_types_pkg::*;
    import memory_types_pkg::*;
    import pe_types_pkg::*;
    import rs_types_pkg::*;

    // IO <-> sequencer.
    wire io_to_seq_t io_data;
    wire seq_to_io_t response;

    wire io_valid;
    wire io_ready;

    wire response_valid;
    wire response_ready;

    // Sequencer <-> shared memory.
    wire mem_req_t mem_req;
    wire mem_rsp_t mem_rsp;

    wire mem_req_valid;
    wire mem_req_ready;

    wire mem_rsp_valid;
    wire mem_rsp_ready;

    // Sequencer -> PE.
    wire seq_to_pe_t pe_cmd [0:N_PE-1];

    wire [N_PE-1:0] pe_cmd_valid;
    wire [N_PE-1:0] pe_cmd_ready;

    wire [N_PE-1:0] pe_done;
    wire pe_status_t pe_status [0:N_PE-1];

    // PE -> reduce + sigmoid.
    wire pe_to_rs_t pe_result [0:N_PE-1];

    wire [N_PE-1:0] pe_result_valid;
    wire [N_PE-1:0] pe_result_ready;

    // Sequencer <-> reduce + sigmoid.
    wire seq_to_rs_t rs_cmd;
    wire rs_to_seq_t rs_result;

    wire rs_start;
    wire rs_ready;

    wire rs_result_valid;
    wire rs_result_ready;

    wire compute_rst_n;

    io_controller #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .UART_BAUD_RATE(UART_BAUD_RATE)
    ) u_io (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),

        .uart_rx_i(uart_rx_i),
        .uart_tx_o(uart_tx_o),

        .io_to_seq_data_o(io_data),
        .io_to_seq_valid_o(io_valid),
        .io_to_seq_ready_i(io_ready),

        .seq_to_io_data_i(response),
        .seq_to_io_valid_i(response_valid),
        .seq_to_io_ready_o(response_ready)
    );

    sequencer #(
        .N_PE(N_PE)
    ) u_seq (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),

        .io_to_seq_data_i(io_data),
        .io_to_seq_valid_i(io_valid),
        .io_to_seq_ready_o(io_ready),

        .seq_to_io_data_o(response),
        .seq_to_io_valid_o(response_valid),
        .seq_to_io_ready_i(response_ready),

        .seq_to_mem_data_o(mem_req),
        .seq_to_mem_valid_o(mem_req_valid),
        .seq_to_mem_ready_i(mem_req_ready),

        .mem_to_seq_data_i(mem_rsp),
        .mem_to_seq_valid_i(mem_rsp_valid),
        .mem_to_seq_ready_o(mem_rsp_ready),

        .seq_to_pe_data_o(pe_cmd),
        .seq_to_pe_valid_o(pe_cmd_valid),
        .seq_to_pe_ready_i(pe_cmd_ready),

        .pe_done_i(pe_done),
        .pe_status_i(pe_status),

        .seq_to_rs_data_o(rs_cmd),
        .seq_to_rs_start_o(rs_start),
        .seq_to_rs_ready_i(rs_ready),

        .rs_to_seq_data_i(rs_result),
        .rs_to_seq_valid_i(rs_result_valid),
        .rs_to_seq_ready_o(rs_result_ready),

        .compute_rst_n_o(compute_rst_n),

        .model_valid_o(model_valid_o),
        .busy_o(busy_o)
    );

    // 4096 * 4 = 16384 байта.
    memory #(
        .MEM_WORDS(4096)
    ) u_shared_memory (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),

        .a_req_data_i(mem_req),
        .a_req_valid_i(mem_req_valid),
        .a_req_ready_o(mem_req_ready),

        .a_rsp_data_o(mem_rsp),
        .a_rsp_valid_o(mem_rsp_valid),
        .a_rsp_ready_i(mem_rsp_ready),

        // ¬торой порт общей пам€ти пока не используетс€.
        .b_req_data_i('0),
        .b_req_valid_i(1'b0),
        .b_req_ready_o(),

        .b_rsp_data_o(),
        .b_rsp_valid_o(),
        .b_rsp_ready_i(1'b1)
    );

    for (genvar p = 0; p < N_PE; p++) begin : g_pe

        pe u_pe (
            .clk_i(clk_i),
            .rst_n_i(compute_rst_n),

            .seq_to_pe_data_i(pe_cmd[p]),
            .seq_to_pe_valid_i(pe_cmd_valid[p]),
            .seq_to_pe_ready_o(pe_cmd_ready[p]),

            .pe_to_reduce_data_o(pe_result[p]),
            .pe_to_reduce_valid_o(pe_result_valid[p]),
            .pe_to_reduce_ready_i(pe_result_ready[p]),

            .pe_done_o(pe_done[p]),
            .pe_status_o(pe_status[p])
        );

    end

    reduce_sigmoid #(
        .N_PE(N_PE),
        .ROM_FILE(ROM_FILE)
    ) u_rs (
        .clk_i(clk_i),
        .rst_n_i(compute_rst_n),

        .seq_to_rs_data_i(rs_cmd),
        .seq_to_rs_start_i(rs_start),
        .seq_to_rs_ready_o(rs_ready),

        .pe_to_rs_data_i(pe_result),
        .pe_to_rs_valid_i(pe_result_valid),
        .pe_to_rs_ready_o(pe_result_ready),

        .rs_to_seq_data_o(rs_result),
        .rs_to_seq_valid_o(rs_result_valid),
        .rs_to_seq_ready_i(rs_result_ready)
    );

endmodule