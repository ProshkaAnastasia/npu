`timescale 1ns/1ps

module sigmoid #(
    parameter ROM_FILE = "sigmoid_lut.mem"
) (
    input logic clk_i,
    input logic rst_n_i,

    input rs_types_pkg::rs_to_seq_t reduce_to_sigmoid_data_i,
    input  logic reduce_to_sigmoid_valid_i,
    output logic reduce_to_sigmoid_ready_o,

    output rs_types_pkg::rs_to_seq_t sigmoid_to_rs_data_o,
    output logic sigmoid_to_rs_valid_o,
    input  logic sigmoid_to_rs_ready_i
);

    import rs_types_pkg::*;

    typedef enum logic [3:0] {
        IDLE,
        DECODE,
        ROM_READ,
        ROM_READY,
        DIFF_REQ,
        DIFF_WAIT,
        MUL_REQ,
        MUL_WAIT,
        FINAL_REQ,
        FINAL_WAIT,
        RESULT
    } state_t;

    state_t state;

    logic [31:0] z_reg;
    logic [31:0] result_reg;
    logic [15:0] neuron_reg;
    rs_status_t status_reg;

    // 257 узлов на [-8; 8], шаг 1/16.
    logic [31:0] rom [0:256];

    logic [8:0] rom_index;
    logic [31:0] rom_a;
    logic [31:0] rom_b;

    logic [31:0] t_reg;
    logic [31:0] diff_reg;
    logic [31:0] product_reg;

    logic [20:0] coord;

    logic [31:0] add_a;
    logic [31:0] add_b;
    logic [31:0] add_result;
    logic add_valid;
    logic add_out_valid;

    logic [31:0] mul_result;
    logic mul_valid;
    logic mul_out_valid;

    logic [2:0] reset_guard;

    initial begin
        $readmemh(ROM_FILE, rom);
    end

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i)
            reset_guard <= 0;
        else
            reset_guard <= {reset_guard[1:0], 1'b1};
    end

    // Точное представление remainder / 4096 в FP32.
    function automatic logic [31:0] fraction_fp32(
        input logic [11:0] remainder
    );
        integer lead;
        logic [31:0] shifted;

        if (remainder == 0)
            return 32'd0;

        lead = 0;

        for (int i = 0; i < 12; i++) begin
            if (remainder[i])
                lead = i;
        end

        shifted = {20'd0, remainder} << (23 - lead);

        return {
            1'b0,
            8'(115 + lead),
            shifted[22:0]
        };
    endfunction

    // Преобразование FP32 в Q5.16 с усечением к нулю.
    // Затем сдвиг диапазона [-8; 8] в [0; 16].
    function automatic logic [20:0] coordinate(
        input logic [31:0] z
    );
        logic [31:0] magnitude;
        integer sh;

        magnitude = 0;

        if (z[30:23] != 0) begin
            sh = 134 - int'({1'b0, z[30:23]});

            if (sh < 24) begin
                magnitude =
                    {8'd0, 1'b1, z[22:0]} >> sh;
            end
        end

        if (z[31])
            return 21'(32'd524288 - magnitude);
        else
            return 21'(32'd524288 + magnitude);
    endfunction

    // Один сумматор используется для b-a и a+product.
    fp32_add_ip u_add (
        .aclk                 (clk_i),
        .aresetn              (rst_n_i),

        .s_axis_a_tvalid      (add_valid),
        .s_axis_a_tdata       (add_a),

        .s_axis_b_tvalid      (add_valid),
        .s_axis_b_tdata       (add_b),

        .m_axis_result_tvalid (add_out_valid),
        .m_axis_result_tdata  (add_result)
    );

    fp32_mul_ip u_mul (
        .aclk                 (clk_i),
        .aresetn              (rst_n_i),

        .s_axis_a_tvalid      (mul_valid),
        .s_axis_a_tdata       (t_reg),

        .s_axis_b_tvalid      (mul_valid),
        .s_axis_b_tdata       (diff_reg),

        .m_axis_result_tvalid (mul_out_valid),
        .m_axis_result_tdata  (mul_result)
    );

    always_comb begin
        reduce_to_sigmoid_ready_o =
            (&reset_guard) && (state == IDLE);

        // В функцию адресации допускаем только |z| <= 8.
        coord = coordinate(
            (z_reg[30:0] <= 31'h41000000)
                ? z_reg
                : 32'd0
        );

        add_a = 0;
        add_b = 0;
        add_valid = 0;

        if (state == DIFF_REQ) begin
            add_a = rom_b;

            // b-a: инверсия знака a.
            add_b = {~rom_a[31], rom_a[30:0]};

            add_valid = 1;
        end

        if (state == FINAL_REQ) begin
            add_a = rom_a;
            add_b = product_reg;
            add_valid = 1;
        end

        mul_valid = (state == MUL_REQ);

        sigmoid_to_rs_data_o.neuron_id = neuron_reg;
        sigmoid_to_rs_data_o.value     = result_reg;
        sigmoid_to_rs_data_o.status    = status_reg;

        sigmoid_to_rs_valid_o = (state == RESULT);
    end

    // Два синхронных чтения ROM.
    always_ff @(posedge clk_i) begin
        if (state == ROM_READ) begin
            rom_a <= rom[rom_index];

            rom_b <= rom[
                (rom_index == 9'd256)
                    ? 9'd256
                    : rom_index + 9'd1
            ];
        end
    end

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            state       <= IDLE;
            z_reg       <= 0;
            result_reg  <= 0;
            neuron_reg  <= 0;
            status_reg  <= RS_OK;

            rom_index   <= 0;
            t_reg       <= 0;
            diff_reg    <= 0;
            product_reg <= 0;

        end else begin
            case (state)

                IDLE: begin
                    if (
                        reduce_to_sigmoid_valid_i &&
                        reduce_to_sigmoid_ready_o
                    ) begin
                        z_reg <= reduce_to_sigmoid_data_i.value;
                        neuron_reg <=
                            reduce_to_sigmoid_data_i.neuron_id;
                        status_reg <=
                            reduce_to_sigmoid_data_i.status;

                        result_reg  <= 0;
                        t_reg       <= 0;
                        diff_reg    <= 0;
                        product_reg <= 0;

                        if (
                            reduce_to_sigmoid_data_i.status != RS_OK
                        ) begin
                            state <= RESULT;

                        end else if (
                            reduce_to_sigmoid_data_i.value[30:23]
                                == 8'hFF
                        ) begin
                            status_reg <= RS_NONFINITE;
                            state <= RESULT;

                        end else begin
                            state <= DECODE;
                        end
                    end
                end

                DECODE: begin
                    if (z_reg[30:0] > 31'h41000000) begin
                        // За пределами [-8; 8].
                        result_reg <= z_reg[31]
                            ? 32'h00000000
                            : 32'h3F800000;

                        state <= RESULT;

                    end else begin
                        rom_index <= coord[20:12];
                        t_reg <= fraction_fp32(coord[11:0]);

                        state <= ROM_READ;
                    end
                end

                ROM_READ: begin
                    state <= ROM_READY;
                end

                ROM_READY: begin
                    if (rom_index == 9'd256) begin
                        // Аргумент ровно +8.
                        result_reg <= rom_a;
                        state <= RESULT;
                    end else begin
                        state <= DIFF_REQ;
                    end
                end

                DIFF_REQ: begin
                    state <= DIFF_WAIT;
                end

                DIFF_WAIT: begin
                    if (add_out_valid) begin
                        if (add_result[30:23] == 8'hFF) begin
                            status_reg <= RS_ARITH_ERROR;
                            state <= RESULT;
                        end else begin
                            diff_reg <= add_result;
                            state <= MUL_REQ;
                        end
                    end
                end

                MUL_REQ: begin
                    state <= MUL_WAIT;
                end

                MUL_WAIT: begin
                    if (mul_out_valid) begin
                        if (mul_result[30:23] == 8'hFF) begin
                            status_reg <= RS_ARITH_ERROR;
                            state <= RESULT;
                        end else begin
                            product_reg <= mul_result;
                            state <= FINAL_REQ;
                        end
                    end
                end

                FINAL_REQ: begin
                    state <= FINAL_WAIT;
                end

                FINAL_WAIT: begin
                    if (add_out_valid) begin
                        if (add_result[30:23] == 8'hFF)
                            status_reg <= RS_ARITH_ERROR;
                        else
                            result_reg <= add_result;

                        state <= RESULT;
                    end
                end

                RESULT: begin
                    if (
                        sigmoid_to_rs_valid_o &&
                        sigmoid_to_rs_ready_i
                    ) begin
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule