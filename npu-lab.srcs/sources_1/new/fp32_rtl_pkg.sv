`timescale 1ns/1ps

package fp32_rtl_pkg;

    localparam logic [31:0] FP32_QNAN = 32'h7FC00000;

    typedef struct packed {
        logic        valid;
        logic [31:0] value;
    } special_t;

    // Субнормальные входы преобразуются в знаковый ноль.
    function automatic logic [31:0] daz(
        input logic [31:0] x
    );
        return (x[30:23] == 0)
            ? {x[31], 31'd0}
            : x;
    endfunction

    // Обработка NaN, Infinity и нулей.
    // Вызывается после daz.
    function automatic special_t special_case(
        input logic [31:0] a,
        input logic [31:0] b,
        input logic multiply
    );
        special_t result;

        logic a_infinite;
        logic b_infinite;
        logic a_zero;
        logic b_zero;

        result = '0;

        a_infinite = (a[30:0] == 31'h7F800000);
        b_infinite = (b[30:0] == 31'h7F800000);

        a_zero = (a[30:0] == 0);
        b_zero = (b[30:0] == 0);

        if (
            (a[30:23] == 8'hFF && a[22:0] != 0) ||
            (b[30:23] == 8'hFF && b[22:0] != 0)
        ) begin
            result.valid = 1'b1;
            result.value = FP32_QNAN;

        end else if (multiply) begin

            if (
                (a_infinite && b_zero) ||
                (b_infinite && a_zero)
            ) begin
                result.valid = 1'b1;
                result.value = FP32_QNAN;

            end else if (a_infinite || b_infinite) begin
                result.valid = 1'b1;
                result.value = {
                    a[31] ^ b[31],
                    8'hFF,
                    23'd0
                };

            end else if (a_zero || b_zero) begin
                result.valid = 1'b1;
                result.value = {
                    a[31] ^ b[31],
                    31'd0
                };
            end

        end else begin

            if (
                a_infinite &&
                b_infinite &&
                a[31] != b[31]
            ) begin
                result.valid = 1'b1;
                result.value = FP32_QNAN;

            end else if (a_infinite || b_infinite) begin
                result.valid = 1'b1;
                result.value = a_infinite ? a : b;

            end else if (a_zero && b_zero) begin
                result.valid = 1'b1;
                result.value = {
                    a[31] & b[31],
                    31'd0
                };

            end else if (a_zero || b_zero) begin
                result.valid = 1'b1;
                result.value = a_zero ? b : a;
            end
        end

        return result;
    endfunction

    // Сдвиг вправо с накоплением отброшенных битов
    // в младшем бите результата - sticky.
    function automatic logic [26:0] shift_jam(
        input logic [26:0] x,
        input integer distance
    );
        logic [26:0] shifted;
        logic sticky;

        shifted = x;
        sticky = 1'b0;

        if (distance >= 27) begin
            shifted = {26'd0, |x};

        end else if (distance > 0) begin
            for (int k = 0; k < 5; k++) begin
                if ((distance & (1 << k)) != 0) begin
                    sticky = |(
                        shifted << (27 - (1 << k))
                    );

                    shifted = shifted >> (1 << k);
                    shifted[0] = shifted[0] | sticky;
                end
            end
        end

        return shifted;
    endfunction

    // Число ведущих нулей в 27-битном значении.
    function automatic logic [4:0] leading_zeros(
        input logic [26:0] x
    );
        logic [26:0] shifted;
        logic [4:0] count;

        shifted = x;
        count = '0;

        if (x == 0)
            return 5'd27;

        if (shifted[26:11] == 0) begin
            count[4] = 1'b1;
            shifted = shifted << 16;
        end

        if (shifted[26:19] == 0) begin
            count[3] = 1'b1;
            shifted = shifted << 8;
        end

        if (shifted[26:23] == 0) begin
            count[2] = 1'b1;
            shifted = shifted << 4;
        end

        if (shifted[26:25] == 0) begin
            count[1] = 1'b1;
            shifted = shifted << 2;
        end

        if (!shifted[26])
            count[0] = 1'b1;

        return count;
    endfunction

    // significand[26:3] - значащая часть.
    // significand[2:0]  - guard, round, sticky.
    //
    // Порядок смещённый, знаковый, чтобы сохранять
    // промежуточное переполнение/потерю порядка.
    function automatic logic [31:0] round_pack(
        input logic sign_bit,
        input logic signed [11:0] exponent,
        input logic [26:0] significand
    );
        integer result_exponent;

        logic [26:0] extended_fraction;
        logic [24:0] rounded_fraction;
        logic increment;

        result_exponent = int'(exponent);
        extended_fraction = significand;

        if (result_exponent < 1) begin
            extended_fraction = shift_jam(
                extended_fraction,
                1 - result_exponent
            );

            result_exponent = 1;
        end

        // Round to nearest, ties to even.
        increment =
            extended_fraction[2] &&
            (
                extended_fraction[1] ||
                extended_fraction[0] ||
                extended_fraction[3]
            );

        rounded_fraction =
            {1'b0, extended_fraction[26:3]} +
            {24'd0, increment};

        if (rounded_fraction[24]) begin
            rounded_fraction = rounded_fraction >> 1;
            result_exponent = result_exponent + 1;
        end

        if (result_exponent >= 255)
            return {sign_bit, 8'hFF, 23'd0};

        // Субнормальный результат сбрасывается в ноль.
        if (!rounded_fraction[23])
            return {sign_bit, 31'd0};

        return {
            sign_bit,
            8'(result_exponent),
            rounded_fraction[22:0]
        };
    endfunction

endpackage