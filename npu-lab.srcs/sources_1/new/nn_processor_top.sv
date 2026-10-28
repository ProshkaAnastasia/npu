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
    import pe_types_pkg::*;
    import memory_types_pkg::*;

    // IO -> секвенсор.
    wire io_to_seq_t io_to_seq_request;
    wire io_to_seq_request_valid;
    wire seq_to_io_request_ready;

    // Секвенсор -> IO.
    wire seq_to_io_t seq_to_io_response;
    wire seq_to_io_response_valid;
    wire io_to_seq_response_ready;

    // Секвенсор -> shared memory.
    wire mem_req_t seq_to_shared_mem_request;
    wire seq_to_shared_mem_request_valid;
    wire shared_mem_to_seq_request_ready;

    // Shared memory -> секвенсор.
    wire mem_rsp_t shared_mem_to_seq_response;
    wire shared_mem_to_seq_response_valid;
    wire seq_to_shared_mem_response_ready;

    // Секвенсор -> PE.
    wire seq_to_pe_t seq_to_pe_command [0:N_PE-1];

    wire [N_PE-1:0] seq_to_pe_command_valid;
    wire [N_PE-1:0] pe_to_seq_command_ready;

    // PE -> секвенсор.
    wire [N_PE-1:0] pe_to_seq_layer_done;
    wire [N_PE-1:0] pe_to_seq_sum_valid;

    wire pe_status_t pe_to_seq_status [0:N_PE-1];

    // PE -> reduce.
    wire [31:0] pe_to_reduce_sum [0:N_PE-1];

    // Секвенсор -> reduce_sigmoid.
    wire [31:0] seq_to_rs_bias;
    wire seq_to_rs_start;
    wire rs_to_seq_start_ready;

    // Reduce_sigmoid -> секвенсор.
    wire [31:0] rs_to_seq_result_data;
    wire rs_types_pkg::rs_status_t rs_to_seq_result_status;
    wire rs_to_seq_result_valid;
    wire seq_to_rs_result_ready;

    wire compute_rst_n;

    io_controller #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .UART_BAUD_RATE(UART_BAUD_RATE)
    ) u_io (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),

        .uart_rx_i(uart_rx_i),
        .uart_tx_o(uart_tx_o),

        .io_to_seq_data_o(io_to_seq_request),
        .io_to_seq_valid_o(io_to_seq_request_valid),
        .io_to_seq_ready_i(seq_to_io_request_ready),

        .seq_to_io_data_i(seq_to_io_response),
        .seq_to_io_valid_i(seq_to_io_response_valid),
        .seq_to_io_ready_o(io_to_seq_response_ready)
    );

    sequencer #(
        .N_PE(N_PE)
    ) u_seq (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),

        .io_to_seq_data_i(io_to_seq_request),
        .io_to_seq_valid_i(io_to_seq_request_valid),
        .io_to_seq_ready_o(seq_to_io_request_ready),

        .seq_to_io_data_o(seq_to_io_response),
        .seq_to_io_valid_o(seq_to_io_response_valid),
        .seq_to_io_ready_i(io_to_seq_response_ready),

        .seq_to_mem_data_o(seq_to_shared_mem_request),
        .seq_to_mem_valid_o(seq_to_shared_mem_request_valid),
        .seq_to_mem_ready_i(shared_mem_to_seq_request_ready),

        .mem_to_seq_data_i(shared_mem_to_seq_response),
        .mem_to_seq_valid_i(shared_mem_to_seq_response_valid),
        .mem_to_seq_ready_o(seq_to_shared_mem_response_ready),

        .seq_to_pe_command_o(seq_to_pe_command),
        .seq_to_pe_command_valid_o(seq_to_pe_command_valid),
        .pe_to_seq_command_ready_i(pe_to_seq_command_ready),

        .pe_to_seq_layer_done_i(pe_to_seq_layer_done),
        .pe_to_seq_sum_valid_i(pe_to_seq_sum_valid),
        .pe_to_seq_status_i(pe_to_seq_status),

        .seq_to_rs_bias_o(seq_to_rs_bias),
        .seq_to_rs_start_o(seq_to_rs_start),
        .rs_to_seq_start_ready_i(rs_to_seq_start_ready),

        .rs_to_seq_result_data_i(rs_to_seq_result_data),
        .rs_to_seq_result_status_i(rs_to_seq_result_status),
        .rs_to_seq_result_valid_i(rs_to_seq_result_valid),
        .seq_to_rs_result_ready_o(seq_to_rs_result_ready),

        .compute_rst_n_o(compute_rst_n),
        .model_valid_o(model_valid_o),
        .busy_o(busy_o)
    );

    // 4096 слов по 4 байта = 16 КиБ.
    memory #(
        .MEM_WORDS(4096)
    ) u_shared_memory (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),

        .a_client_to_mem_request_data_i(
            seq_to_shared_mem_request
        ),
        .a_client_to_mem_request_valid_i(
            seq_to_shared_mem_request_valid
        ),
        .a_mem_to_client_request_ready_o(
            shared_mem_to_seq_request_ready
        ),

        .a_mem_to_client_response_data_o(
            shared_mem_to_seq_response
        ),
        .a_mem_to_client_response_valid_o(
            shared_mem_to_seq_response_valid
        ),
        .a_client_to_mem_response_ready_i(
            seq_to_shared_mem_response_ready
        ),

        // Второй порт общей памяти не используется.
        .b_client_to_mem_request_data_i('0),
        .b_client_to_mem_request_valid_i(1'b0),
        .b_mem_to_client_request_ready_o(),

        .b_mem_to_client_response_data_o(),
        .b_mem_to_client_response_valid_o(),
        .b_client_to_mem_response_ready_i(1'b1)
    );

    for (genvar p = 0; p < N_PE; p++) begin : g_pe

        pe u_pe (
            .clk_i(clk_i),
            .rst_n_i(compute_rst_n),

            .seq_to_pe_command_i(seq_to_pe_command[p]),
            .seq_to_pe_command_valid_i(
                seq_to_pe_command_valid[p]
            ),
            .pe_to_seq_command_ready_o(
                pe_to_seq_command_ready[p]
            ),

            .pe_to_reduce_sum_o(pe_to_reduce_sum[p]),
            .pe_to_seq_sum_valid_o(pe_to_seq_sum_valid[p]),

            .pe_to_seq_layer_done_o(pe_to_seq_layer_done[p]),
            .pe_to_seq_status_o(pe_to_seq_status[p])
        );

    end

    reduce_sigmoid #(
        .N_PE(N_PE),
        .ROM_FILE(ROM_FILE)
    ) u_rs (
        .clk_i(clk_i),
        .rst_n_i(compute_rst_n),

        .pe_sums_i(pe_to_reduce_sum),

        .seq_bias_i(seq_to_rs_bias),
        .seq_start_i(seq_to_rs_start),
        .seq_start_ready_o(rs_to_seq_start_ready),

        .seq_result_data_o(rs_to_seq_result_data),
        .seq_result_status_o(rs_to_seq_result_status),
        .seq_result_valid_o(rs_to_seq_result_valid),
        .seq_result_ready_i(seq_to_rs_result_ready)
    );

endmodule