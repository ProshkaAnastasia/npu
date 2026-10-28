`timescale 1ns/1ps

module sigmoid #(
    parameter ROM_FILE = "sigmoid_lut.mem"
) (
    input logic clk_i,
    input logic rst_n_i,

    input logic [31:0] input_data_i,
    input rs_types_pkg::rs_status_t input_status_i,
    input  logic input_valid_i,
    output logic input_ready_o,

    output logic [31:0] result_data_o,
    output rs_types_pkg::rs_status_t result_status_o,
    output logic result_valid_o,
    input  logic result_ready_i
);

    import rs_types_pkg::*;

    typedef enum logic [3:0] {
        WAIT_INPUT,
        PREPARE_LOOKUP,
        READ_TABLE,
        WAIT_TABLE_DATA,
        ISSUE_DIFFERENCE,
        WAIT_DIFFERENCE,
        ISSUE_MULTIPLY,
        WAIT_MULTIPLY,
        ISSUE_FINAL_ADD,
        WAIT_FINAL_ADD,
        RESULT_PENDING
    } state_t;

    state_t state;

    logic [31:0] input_value_reg;
    logic [31:0] result_value_reg;

    rs_status_t result_status_reg;

    // 257 значений sigmoid на [-8; 8], шаг 1/16.
    logic [31:0] lookup_table [0:256];

    logic [8:0] table_index_reg;

    logic [31:0] left_value_reg;
    logic [31:0] right_value_reg;

    logic [31:0] interpolation_weight_reg;
    logic [31:0] interval_difference_reg;
    logic [31:0] weighted_difference_reg;

    logic [20:0] table_coordinate;

    logic [31:0] add_operand_a;
    logic [31:0] add_operand_b;
    logic [31:0] add_result;

    logic add_request_valid;
    logic add_result_valid;

    logic [31:0] multiply_result;

    logic multiply_request_valid;
    logic multiply_result_valid;

    logic [2:0] reset_release_pipe;

    initial begin
        $readmemh(ROM_FILE, lookup_table);
    end

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i)
            reset_release_pipe <= '0;
        else
            reset_release_pipe <= {
                reset_release_pipe[1:0],
                1'b1
            };
    end

    // Точное преобразование fraction_q12 / 4096 в FP32.
    function automatic logic [31:0] fraction_to_fp32(
        input logic [11:0] fraction_q12
    );
        integer leading_bit;
        logic [31:0] shifted_fraction;

        if (fraction_q12 == 0)
            return 32'd0;

        leading_bit = 0;

        for (int i = 0; i < 12; i++) begin
            if (fraction_q12[i])
                leading_bit = i;
        end

        shifted_fraction =
            {20'd0, fraction_q12} << (23 - leading_bit);

        return {
            1'b0,
            8'(115 + leading_bit),
            shifted_fraction[22:0]
        };
    endfunction

    // Допустимый аргумент: [-8; 8].
    // Преобразование в Q5.16 и сдвиг диапазона на +8.
    function automatic logic [20:0] fp32_to_table_coordinate(
        input logic [31:0] value
    );
        logic [31:0] magnitude_q16;
        integer shift_count;

        magnitude_q16 = '0;
        shift_count = 0;

        if (value[30:23] != 0) begin
            shift_count =
                134 - int'({1'b0, value[30:23]});

            if (shift_count < 24) begin
                magnitude_q16 =
                    {8'd0, 1'b1, value[22:0]} >> shift_count;
            end
        end

        if (value[31])
            return 21'(32'd524288 - magnitude_q16);
        else
            return 21'(32'd524288 + magnitude_q16);
    endfunction

    fp32_add_ip u_add (
        .aclk(clk_i),
        .aresetn(rst_n_i),

        .s_axis_a_tvalid(add_request_valid),
        .s_axis_a_tdata(add_operand_a),

        .s_axis_b_tvalid(add_request_valid),
        .s_axis_b_tdata(add_operand_b),

        .m_axis_result_tvalid(add_result_valid),
        .m_axis_result_tdata(add_result)
    );

    fp32_mul_ip u_multiply (
        .aclk(clk_i),
        .aresetn(rst_n_i),

        .s_axis_a_tvalid(multiply_request_valid),
        .s_axis_a_tdata(interpolation_weight_reg),

        .s_axis_b_tvalid(multiply_request_valid),
        .s_axis_b_tdata(interval_difference_reg),

        .m_axis_result_tvalid(multiply_result_valid),
        .m_axis_result_tdata(multiply_result)
    );

    assign input_ready_o =
        (&reset_release_pipe) &&
        (state == WAIT_INPUT);

    assign result_data_o = result_value_reg;
    assign result_status_o = result_status_reg;
    assign result_valid_o = (state == RESULT_PENDING);

    always_comb begin
        table_coordinate = fp32_to_table_coordinate(
            (input_value_reg[30:0] <= 31'h41000000)
                ? input_value_reg
                : 32'd0
        );

        add_operand_a = '0;
        add_operand_b = '0;
        add_request_valid = 1'b0;

        if (state == ISSUE_DIFFERENCE) begin
            add_operand_a = right_value_reg;

            add_operand_b = {
                ~left_value_reg[31],
                left_value_reg[30:0]
            };

            add_request_valid = 1'b1;
        end

        if (state == ISSUE_FINAL_ADD) begin
            add_operand_a = left_value_reg;
            add_operand_b = weighted_difference_reg;
            add_request_valid = 1'b1;
        end

        multiply_request_valid =
            (state == ISSUE_MULTIPLY);
    end

    // Синхронное чтение двух соседних узлов таблицы.
    always_ff @(posedge clk_i) begin
        if (state == READ_TABLE) begin
            left_value_reg <= lookup_table[table_index_reg];

            right_value_reg <= lookup_table[
                (table_index_reg == 9'd256)
                    ? 9'd256
                    : table_index_reg + 9'd1
            ];
        end
    end

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            state <= WAIT_INPUT;

            input_value_reg <= '0;
            result_value_reg <= '0;
            result_status_reg <= RS_OK;

            table_index_reg <= '0;

            interpolation_weight_reg <= '0;
            interval_difference_reg <= '0;
            weighted_difference_reg <= '0;

        end else begin
            case (state)

                WAIT_INPUT: begin
                    if (input_valid_i && input_ready_o) begin
                        input_value_reg <= input_data_i;
                        result_status_reg <= input_status_i;

                        result_value_reg <= '0;
                        interpolation_weight_reg <= '0;
                        interval_difference_reg <= '0;
                        weighted_difference_reg <= '0;

                        if (input_status_i != RS_OK) begin
                            state <= RESULT_PENDING;

                        end else if (
                            input_data_i[30:23] == 8'hFF
                        ) begin
                            result_status_reg <= RS_NONFINITE;
                            state <= RESULT_PENDING;

                        end else begin
                            state <= PREPARE_LOOKUP;
                        end
                    end
                end

                PREPARE_LOOKUP: begin
                    if (
                        input_value_reg[30:0] > 31'h41000000
                    ) begin
                        // Насыщение за пределами [-8; 8].
                        result_value_reg <= input_value_reg[31]
                            ? 32'h00000000
                            : 32'h3F800000;

                        state <= RESULT_PENDING;

                    end else begin
                        table_index_reg <=
                            table_coordinate[20:12];

                        interpolation_weight_reg <=
                            fraction_to_fp32(
                                table_coordinate[11:0]
                            );

                        state <= READ_TABLE;
                    end
                end

                READ_TABLE: begin
                    state <= WAIT_TABLE_DATA;
                end

                WAIT_TABLE_DATA: begin
                    if (table_index_reg == 9'd256) begin
                        result_value_reg <= left_value_reg;
                        state <= RESULT_PENDING;
                    end else begin
                        state <= ISSUE_DIFFERENCE;
                    end
                end

                ISSUE_DIFFERENCE: begin
                    state <= WAIT_DIFFERENCE;
                end

                WAIT_DIFFERENCE: begin
                    if (add_result_valid) begin
                        if (add_result[30:23] == 8'hFF) begin
                            result_status_reg <= RS_ARITH_ERROR;
                            state <= RESULT_PENDING;
                        end else begin
                            interval_difference_reg <= add_result;
                            state <= ISSUE_MULTIPLY;
                        end
                    end
                end

                ISSUE_MULTIPLY: begin
                    state <= WAIT_MULTIPLY;
                end

                WAIT_MULTIPLY: begin
                    if (multiply_result_valid) begin
                        if (
                            multiply_result[30:23] == 8'hFF
                        ) begin
                            result_status_reg <= RS_ARITH_ERROR;
                            state <= RESULT_PENDING;
                        end else begin
                            weighted_difference_reg <=
                                multiply_result;

                            state <= ISSUE_FINAL_ADD;
                        end
                    end
                end

                ISSUE_FINAL_ADD: begin
                    state <= WAIT_FINAL_ADD;
                end

                WAIT_FINAL_ADD: begin
                    if (add_result_valid) begin
                        if (add_result[30:23] == 8'hFF)
                            result_status_reg <= RS_ARITH_ERROR;
                        else
                            result_value_reg <= add_result;

                        state <= RESULT_PENDING;
                    end
                end

                RESULT_PENDING: begin
                    if (result_valid_o && result_ready_i)
                        state <= WAIT_INPUT;
                end

                default: begin
                    state <= WAIT_INPUT;
                end

            endcase
        end
    end

endmodule