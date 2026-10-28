`timescale 1ns/1ps

module reduce #(
    parameter int N_PE = 6
) (
    input logic clk_i,
    input logic rst_n_i,

    input logic [31:0] pe_sums_i [0:N_PE-1],
    input logic [31:0] bias_i,

    input  logic start_i,
    output logic start_ready_o,

    output logic [31:0] result_data_o,
    output rs_types_pkg::rs_status_t result_status_o,
    output logic result_valid_o,
    input  logic result_ready_i
);

    import rs_types_pkg::*;

    typedef enum logic [2:0] {
        WAIT_START,
        LOAD_FIRST_SUM,
        ISSUE_ADD,
        WAIT_ADD_RESULT,
        RESULT_PENDING
    } state_t;

    state_t state;

    logic [31:0] partial_sum_regs [0:N_PE-1];

    logic [31:0] bias_value_reg;
    logic [31:0] accumulator_reg;
    logic [31:0] result_value_reg;

    rs_status_t result_status_reg;

    integer partial_sum_index;

    logic adding_bias_phase;
    logic start_accepted;

    wire [N_PE-1:0] finite_parts;

    logic [2:0] reset_release_pipe;

    logic [31:0] add_b;
    logic [31:0] add_result;

    logic add_request_valid;
    logic add_result_valid;

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i)
            reset_release_pipe <= '0;
        else
            reset_release_pipe <= {
                reset_release_pipe[1:0],
                1'b1
            };
    end

    assign start_ready_o =
        (&reset_release_pipe) &&
        (state == WAIT_START);

    assign start_accepted =
        start_i && start_ready_o;

    for (genvar p = 0; p < N_PE; p++) begin : g_parts

        assign finite_parts[p] =
            pe_sums_i[p][30:23] != 8'hFF;

        always_ff @(posedge clk_i or negedge rst_n_i) begin
            if (!rst_n_i)
                partial_sum_regs[p] <= '0;
            else if (start_accepted)
                partial_sum_regs[p] <= pe_sums_i[p];
        end

    end

    assign add_b = adding_bias_phase
        ? bias_value_reg
        : partial_sum_regs[partial_sum_index];

    assign add_request_valid = (state == ISSUE_ADD);

    fp32_add_ip u_add (
        .aclk(clk_i),
        .aresetn(rst_n_i),

        .s_axis_a_tvalid(add_request_valid),
        .s_axis_a_tdata(accumulator_reg),

        .s_axis_b_tvalid(add_request_valid),
        .s_axis_b_tdata(add_b),

        .m_axis_result_tvalid(add_result_valid),
        .m_axis_result_tdata(add_result)
    );

    assign result_data_o = result_value_reg;
    assign result_status_o = result_status_reg;
    assign result_valid_o = (state == RESULT_PENDING);

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            state <= WAIT_START;

            bias_value_reg <= '0;
            accumulator_reg <= '0;
            result_value_reg <= '0;
            result_status_reg <= RS_OK;

            partial_sum_index <= 0;
            adding_bias_phase <= 1'b0;

        end else begin
            case (state)

                WAIT_START: begin
                    if (start_accepted) begin
                        bias_value_reg <= bias_i;

                        accumulator_reg <= '0;
                        result_value_reg <= '0;
                        result_status_reg <= RS_OK;

                        partial_sum_index <=
                            (N_PE > 1) ? 1 : 0;

                        adding_bias_phase <= (N_PE == 1);

                        if (
                            !(&finite_parts) ||
                            bias_i[30:23] == 8'hFF
                        ) begin
                            result_status_reg <= RS_NONFINITE;
                            state <= RESULT_PENDING;
                        end else begin
                            state <= LOAD_FIRST_SUM;
                        end
                    end
                end

                LOAD_FIRST_SUM: begin
                    // Первую сумму загружаем без сложения.
                    accumulator_reg <= partial_sum_regs[0];
                    state <= ISSUE_ADD;
                end

                ISSUE_ADD: begin
                    state <= WAIT_ADD_RESULT;
                end

                WAIT_ADD_RESULT: begin
                    if (add_result_valid) begin
                        if (add_result[30:23] == 8'hFF) begin
                            result_status_reg <= RS_ARITH_ERROR;
                            state <= RESULT_PENDING;

                        end else if (adding_bias_phase) begin
                            result_value_reg <= add_result;
                            state <= RESULT_PENDING;

                        end else begin
                            accumulator_reg <= add_result;

                            if (partial_sum_index == N_PE - 1)
                                adding_bias_phase <= 1'b1;
                            else
                                partial_sum_index <=
                                    partial_sum_index + 1;

                            state <= ISSUE_ADD;
                        end
                    end
                end

                RESULT_PENDING: begin
                    if (result_valid_o && result_ready_i)
                        state <= WAIT_START;
                end

                default: begin
                    state <= WAIT_START;
                end

            endcase
        end
    end

endmodule