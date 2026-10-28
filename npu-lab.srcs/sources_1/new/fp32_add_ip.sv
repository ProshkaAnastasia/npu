`timescale 1ns/1ps

module fp32_add_ip (
    input logic aclk,
    input logic aresetn,

    input logic s_axis_a_tvalid,
    input logic [31:0] s_axis_a_tdata,

    input logic s_axis_b_tvalid,
    input logic [31:0] s_axis_b_tdata,

    output logic m_axis_result_tvalid,
    output logic [31:0] m_axis_result_tdata
);

    import fp32_rtl_pkg::*;

    logic [31:0] operand_a;
    logic [31:0] operand_b;

    logic swap_operands;

    logic [31:0] larger_operand;
    logic [31:0] smaller_operand;

    special_t input_special;
    special_t stage1_special;
    special_t stage2_special;
    special_t stage3_special;

    logic stage1_valid;
    logic stage2_valid;
    logic stage3_valid;

    logic stage1_sign;
    logic stage1_subtract;

    logic [7:0] stage1_exponent;
    logic [7:0] stage1_exponent_distance;

    logic [23:0] stage1_larger_fraction;
    logic [23:0] stage1_smaller_fraction;

    logic [26:0] larger_extended;
    logic [26:0] smaller_aligned;

    logic [27:0] extended_sum;

    logic [26:0] arithmetic_fraction;
    logic signed [11:0] arithmetic_exponent;

    logic stage2_sign;
    logic signed [11:0] stage2_exponent;
    logic [26:0] stage2_fraction;

    logic stage3_sign;
    logic signed [11:0] stage3_exponent;
    logic [26:0] stage3_fraction;

    logic [4:0] leading_zero_count;
    integer normalization_shift;

    always @* begin
        operand_a = daz(s_axis_a_tdata);
        operand_b = daz(s_axis_b_tdata);

        input_special = special_case(
            operand_a,
            operand_b,
            1'b0
        );

        swap_operands =
            operand_a[30:0] < operand_b[30:0];

        larger_operand = swap_operands
            ? operand_b
            : operand_a;

        smaller_operand = swap_operands
            ? operand_a
            : operand_b;

        larger_extended = {
            stage1_larger_fraction,
            3'b000
        };

        smaller_aligned = shift_jam(
            {stage1_smaller_fraction, 3'b000},
            int'({1'b0, stage1_exponent_distance})
        );

        extended_sum =
            {1'b0, larger_extended} +
            {1'b0, smaller_aligned};

        arithmetic_exponent =
            $signed({4'd0, stage1_exponent});

        if (stage1_subtract) begin
            arithmetic_fraction =
                larger_extended - smaller_aligned;

        end else if (extended_sum[27]) begin
            arithmetic_fraction = extended_sum[27:1];

            arithmetic_fraction[0] =
                arithmetic_fraction[0] |
                extended_sum[0];

            arithmetic_exponent =
                arithmetic_exponent + 12'sd1;

        end else begin
            arithmetic_fraction = extended_sum[26:0];
        end

        leading_zero_count =
            leading_zeros(stage2_fraction);

        normalization_shift =
            int'({1'b0, leading_zero_count});

        if (
            normalization_shift >
            int'(stage2_exponent) - 1
        )
            normalization_shift =
                int'(stage2_exponent) - 1;

        if (normalization_shift < 0)
            normalization_shift = 0;
    end

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            stage1_valid <= 1'b0;
            stage2_valid <= 1'b0;
            stage3_valid <= 1'b0;

            stage1_special <= '0;
            stage2_special <= '0;
            stage3_special <= '0;

            stage1_sign <= 1'b0;
            stage1_subtract <= 1'b0;

            stage1_exponent <= '0;
            stage1_exponent_distance <= '0;

            stage1_larger_fraction <= '0;
            stage1_smaller_fraction <= '0;

            stage2_sign <= 1'b0;
            stage2_exponent <= '0;
            stage2_fraction <= '0;

            stage3_sign <= 1'b0;
            stage3_exponent <= '0;
            stage3_fraction <= '0;

            m_axis_result_tvalid <= 1'b0;
            m_axis_result_tdata <= '0;

        end else begin

            // Стадия 1: приём и распаковка операндов.
            stage1_valid <=
                s_axis_a_tvalid && s_axis_b_tvalid;

            if (s_axis_a_tvalid && s_axis_b_tvalid) begin
                stage1_special <= input_special;

                stage1_sign <= larger_operand[31];

                stage1_subtract <=
                    operand_a[31] ^ operand_b[31];

                stage1_exponent <=
                    larger_operand[30:23];

                stage1_exponent_distance <=
                    larger_operand[30:23] -
                    smaller_operand[30:23];

                stage1_larger_fraction <= {
                    1'b1,
                    larger_operand[22:0]
                };

                stage1_smaller_fraction <= {
                    1'b1,
                    smaller_operand[22:0]
                };
            end

            // Стадия 2: выравнивание и арифметика.
            stage2_valid <= stage1_valid;

            if (stage1_valid) begin
                stage2_special <= stage1_special;

                stage2_sign <= stage1_sign;
                stage2_exponent <= arithmetic_exponent;
                stage2_fraction <= arithmetic_fraction;

                if (
                    !stage1_special.valid &&
                    arithmetic_fraction == 0
                ) begin
                    stage2_special.valid <= 1'b1;
                    stage2_special.value <= 32'd0;
                end
            end

            // Стадия 3: нормализация.
            stage3_valid <= stage2_valid;

            if (stage2_valid) begin
                stage3_special <= stage2_special;
                stage3_sign <= stage2_sign;

                stage3_fraction <=
                    stage2_fraction << normalization_shift;

                stage3_exponent <=
                    stage2_exponent -
                    12'(normalization_shift);
            end

            // Стадия 4: округление и упаковка.
            m_axis_result_tvalid <= stage3_valid;

            if (stage3_valid) begin
                if (stage3_special.valid)
                    m_axis_result_tdata <=
                        stage3_special.value;
                else
                    m_axis_result_tdata <= round_pack(
                        stage3_sign,
                        stage3_exponent,
                        stage3_fraction
                    );
            end
        end
    end

endmodule