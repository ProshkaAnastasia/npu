`timescale 1ns/1ps

module fp32_mul_ip (
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

    special_t input_special;
    special_t stage1_special;
    special_t stage2_special;

    logic stage1_valid;
    logic stage2_valid;

    logic stage1_sign;
    logic signed [11:0] stage1_exponent;
    logic [47:0] stage1_product;

    logic stage2_sign;
    logic signed [11:0] stage2_exponent;
    logic [26:0] stage2_fraction;

    logic [26:0] normalized_fraction;

    always @* begin
        operand_a = daz(s_axis_a_tdata);
        operand_b = daz(s_axis_b_tdata);

        input_special = special_case(
            operand_a,
            operand_b,
            1'b1
        );

        if (stage1_product[47]) begin
            normalized_fraction = stage1_product[47:21];

            normalized_fraction[0] =
                normalized_fraction[0] |
                (|stage1_product[20:0]);

        end else begin
            normalized_fraction = stage1_product[46:20];

            normalized_fraction[0] =
                normalized_fraction[0] |
                (|stage1_product[19:0]);
        end
    end

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            stage1_valid <= 1'b0;
            stage2_valid <= 1'b0;

            stage1_special <= '0;
            stage2_special <= '0;

            stage1_sign <= 1'b0;
            stage1_exponent <= '0;
            stage1_product <= '0;

            stage2_sign <= 1'b0;
            stage2_exponent <= '0;
            stage2_fraction <= '0;

            m_axis_result_tvalid <= 1'b0;
            m_axis_result_tdata <= '0;

        end else begin

            // Стадия 1: произведение значащих частей 24x24.
            stage1_valid <=
                s_axis_a_tvalid && s_axis_b_tvalid;

            if (s_axis_a_tvalid && s_axis_b_tvalid) begin
                stage1_special <= input_special;

                stage1_sign <=
                    operand_a[31] ^ operand_b[31];

                stage1_exponent <=
                    $signed({4'd0, operand_a[30:23]}) +
                    $signed({4'd0, operand_b[30:23]}) -
                    12'sd127;

                stage1_product <=
                    {1'b1, operand_a[22:0]} *
                    {1'b1, operand_b[22:0]};
            end

            // Стадия 2: нормализация.
            stage2_valid <= stage1_valid;

            if (stage1_valid) begin
                stage2_special <= stage1_special;
                stage2_sign <= stage1_sign;

                stage2_exponent <=
                    stage1_exponent +
                    (
                        stage1_product[47]
                            ? 12'sd1
                            : 12'sd0
                    );

                stage2_fraction <= normalized_fraction;
            end

            // Стадия 3: округление и упаковка.
            m_axis_result_tvalid <= stage2_valid;

            if (stage2_valid) begin
                if (stage2_special.valid)
                    m_axis_result_tdata <=
                        stage2_special.value;
                else
                    m_axis_result_tdata <= round_pack(
                        stage2_sign,
                        stage2_exponent,
                        stage2_fraction
                    );
            end
        end
    end

endmodule