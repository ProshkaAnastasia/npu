`timescale 1ns/1ps

module reduce #(
    parameter int N_PE = 6
) (
    input logic clk_i,
    input logic rst_n_i,

    input rs_types_pkg::seq_to_rs_t seq_to_reduce_data_i,
    input  logic seq_to_reduce_start_i,
    output logic seq_to_reduce_ready_o,

    input rs_types_pkg::pe_to_rs_t
        pe_to_reduce_data_i [0:N_PE-1],

    input  logic [N_PE-1:0] pe_to_reduce_valid_i,
    output logic [N_PE-1:0] pe_to_reduce_ready_o,

    output rs_types_pkg::rs_to_seq_t reduce_to_sigmoid_data_o,
    output logic reduce_to_sigmoid_valid_o,
    input  logic reduce_to_sigmoid_ready_i
);

    import rs_types_pkg::*;

    typedef enum logic [2:0] {
        IDLE,
        LOAD_FIRST,
        ADD_REQ,
        ADD_WAIT,
        RESULT
    } state_t;

    state_t state;

    // По одному регистру на каждый PE.
    logic [31:0] parts [0:N_PE-1];

    logic [31:0] bias_reg;
    logic [31:0] acc_reg;
    logic [31:0] result_reg;
    logic [15:0] neuron_reg;

    rs_status_t status_reg;

    integer part_index;
    logic adding_bias;
    logic accept;

    wire [N_PE-1:0] id_matches;
    wire [N_PE-1:0] finite_parts;

    // Пауза после снятия сброса для арифметического IP.
    logic [2:0] reset_guard;

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i)
            reset_guard <= 0;
        else
            reset_guard <= {reset_guard[1:0], 1'b1};
    end

    assign seq_to_reduce_ready_o =
        (&reset_guard) &&
        (state == IDLE) &&
        (&pe_to_reduce_valid_i);

    assign accept =
        seq_to_reduce_start_i &&
        seq_to_reduce_ready_o;

    // Все PE получают подтверждение одновременно.
    assign pe_to_reduce_ready_o = {N_PE{accept}};

    for (genvar p = 0; p < N_PE; p++) begin : g_parts

        wire pe_to_rs_t item;

        assign item = pe_to_reduce_data_i[p];

        assign id_matches[p] =
            item.neuron_id == seq_to_reduce_data_i.neuron_id;

        assign finite_parts[p] =
            item.value[30:23] != 8'hFF;

        always_ff @(posedge clk_i or negedge rst_n_i) begin
            if (!rst_n_i)
                parts[p] <= 0;
            else if (accept)
                parts[p] <= item.value;
        end

    end

    // Интерфейс FP32-сумматора.
    logic [31:0] add_b;
    logic [31:0] add_result;
    logic add_valid;
    logic add_out_valid;

    assign add_b = adding_bias
        ? bias_reg
        : parts[part_index];

    // Один такт запуска, затем ожидание результата.
    assign add_valid = (state == ADD_REQ);

    fp32_add_ip u_add (
        .aclk                  (clk_i),
        .aresetn               (rst_n_i),

        .s_axis_a_tvalid       (add_valid),
        .s_axis_a_tdata        (acc_reg),

        .s_axis_b_tvalid       (add_valid),
        .s_axis_b_tdata        (add_b),

        .m_axis_result_tvalid  (add_out_valid),
        .m_axis_result_tdata   (add_result)
    );

    always_comb begin
        reduce_to_sigmoid_data_o.neuron_id = neuron_reg;
        reduce_to_sigmoid_data_o.value     = result_reg;
        reduce_to_sigmoid_data_o.status    = status_reg;

        reduce_to_sigmoid_valid_o = (state == RESULT);
    end

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            state        <= IDLE;
            bias_reg     <= 0;
            acc_reg      <= 0;
            result_reg   <= 0;
            neuron_reg   <= 0;
            part_index   <= 0;
            adding_bias  <= 0;
            status_reg   <= RS_OK;

        end else begin
            case (state)

                IDLE: begin
                    if (accept) begin
                        neuron_reg  <= seq_to_reduce_data_i.neuron_id;
                        bias_reg    <= seq_to_reduce_data_i.bias;

                        acc_reg     <= 0;
                        result_reg  <= 0;
                        status_reg  <= RS_OK;

                        part_index  <= (N_PE > 1) ? 1 : 0;
                        adding_bias <= (N_PE == 1);

                        if (!(&id_matches)) begin
                            status_reg <= RS_BAD_NEURON;
                            state <= RESULT;

                        end else if (
                            !(&finite_parts) ||
                            seq_to_reduce_data_i.bias[30:23] == 8'hFF
                        ) begin
                            status_reg <= RS_NONFINITE;
                            state <= RESULT;

                        end else begin
                            state <= LOAD_FIRST;
                        end
                    end
                end

                LOAD_FIRST: begin
                    // Первую часть загружаем без сложения.
                    acc_reg <= parts[0];
                    state <= ADD_REQ;
                end

                ADD_REQ: begin
                    state <= ADD_WAIT;
                end

                ADD_WAIT: begin
                    if (add_out_valid) begin
                        if (add_result[30:23] == 8'hFF) begin
                            status_reg <= RS_ARITH_ERROR;
                            state <= RESULT;

                        end else if (adding_bias) begin
                            result_reg <= add_result;
                            state <= RESULT;

                        end else begin
                            acc_reg <= add_result;

                            if (part_index == N_PE - 1)
                                adding_bias <= 1'b1;
                            else
                                part_index <= part_index + 1;

                            state <= ADD_REQ;
                        end
                    end
                end

                RESULT: begin
                    if (
                        reduce_to_sigmoid_valid_o &&
                        reduce_to_sigmoid_ready_i
                    ) begin
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule