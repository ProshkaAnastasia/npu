`timescale 1ns/1ps

module pe #(
    // Количество 32-битных слов локальной памяти.
    // 2048 слов = 8 КиБ.
    parameter int MEM_WORDS = 2048
) (
    input logic clk_i,
    input logic rst_n_i,

    // Команды секвенсора.
    input pe_types_pkg::seq_to_pe_t seq_to_pe_data_i,
    input  logic seq_to_pe_valid_i,
    output logic seq_to_pe_ready_o,

    // Частичная сумма напрямую в reduce.
    output rs_types_pkg::pe_to_rs_t pe_to_reduce_data_o,
    output logic pe_to_reduce_valid_o,
    input  logic pe_to_reduce_ready_i,

    // Все суммы слоя вычислены и сохранены.
    output logic pe_done_o,

    // Код ошибки; PE_OK при отсутствии ошибки.
    output pe_types_pkg::pe_status_t pe_status_o
);

    import pe_types_pkg::*;
    import memory_types_pkg::*;

    localparam logic [31:0] MEM_BYTES =
        $unsigned(MEM_WORDS) << 2;

    localparam logic [31:0] WORD_LIMIT =
        $unsigned(MEM_WORDS);

    localparam logic [63:0] MEM_CAPACITY =
        {32'd0, MEM_BYTES};

    typedef enum logic [3:0] {
        CONTROL,
        CLEAR_SUMS,
        MAC_READ,
        PAIR_WAIT,
        MUL_WAIT,
        ADD_REQ,
        ADD_WAIT,
        STORE,
        STORE_WAIT,
        OUT_READ,
        OUT_WAIT,
        OUT_VALID,
        HOLD,
        ERROR
    } state_t;

    state_t state;

    // --------------------------------------------------------
    // Конфигурация и управляющие регистры.
    // --------------------------------------------------------

    pe_cfg_t cfg_reg;
    pe_status_t status_reg;

    logic configured;
    logic done_reg;

    logic [2:0] reset_guard;

    logic command_fire;
    logic write_addr_ok;
    logic cfg_legal;
    logic agu_addr_ok;

    logic [31:0] clear_addr;
    logic [31:0] out_addr;

    logic [31:0] x_limit;
    logic [31:0] s_limit;

    logic [15:0] last_neuron;
    logic [15:0] clear_index;
    logic [15:0] out_neuron;

    // Выходной регистр и регистры арифметического тракта.
    logic [31:0] output_value;
    logic [31:0] old_sum;
    logic [31:0] product;
    logic [31:0] new_sum;

    logic sum_seen;
    logic product_seen;

    // --------------------------------------------------------
    // Интерфейсы двух портов локальной памяти.
    // --------------------------------------------------------

    mem_req_t a_req;
    mem_req_t b_req;

    mem_rsp_t a_rsp;
    mem_rsp_t b_rsp;

    logic a_valid;
    logic b_valid;

    logic a_ready;
    logic b_ready;

    logic a_rsp_valid;
    logic b_rsp_valid;

    // --------------------------------------------------------
    // Интерфейс AGU.
    // --------------------------------------------------------

    agu_to_pe_t agu_data;

    logic agu_valid;
    logic agu_load;
    logic agu_advance;

    // --------------------------------------------------------
    // Интерфейсы арифметических IP.
    // --------------------------------------------------------

    logic mul_launch;
    logic mul_out_valid;

    logic add_launch;
    logic add_out_valid;

    logic [31:0] mul_result;
    logic [31:0] add_result;

    // --------------------------------------------------------
    // Проверки адресов и конфигурации.
    // --------------------------------------------------------

    function automatic logic physical_addr_ok(
        input logic [31:0] addr
    );
        return
            addr[1:0] == 2'b00 &&
            addr < MEM_BYTES;
    endfunction

    function automatic logic overlap(
        input logic [63:0] lo_a,
        input logic [63:0] hi_a,
        input logic [63:0] lo_b,
        input logic [63:0] hi_b
    );
        return
            lo_a < hi_a &&
            lo_b < hi_b &&
            lo_a < hi_b &&
            lo_b < hi_a;
    endfunction

    // Проверка размещения активных областей.
    // Правильность mac_count и шагов обеспечивает секвенсор.
    function automatic logic config_ok(
        input pe_cfg_t c
    );
        logic [63:0] wb;
        logic [63:0] xb;
        logic [63:0] sb;

        logic [63:0] we;
        logic [63:0] xe;
        logic [63:0] se;

        wb = {32'd0, c.w_base};
        xb = {32'd0, c.x_base};
        sb = {32'd0, c.s_base};

        we = wb + ({32'd0, c.mac_count} << 2);
        xe = xb + ({48'd0, c.n_in} << 2);
        se = sb + ({48'd0, c.n_out} << 2);

        return
            c.n_in != 0 &&
            c.n_out != 0 &&

            c.w_base[1:0] == 2'b00 &&
            c.x_base[1:0] == 2'b00 &&
            c.s_base[1:0] == 2'b00 &&

            c.mac_count <= WORD_LIMIT &&

            we <= MEM_CAPACITY &&
            xe <= MEM_CAPACITY &&
            se <= MEM_CAPACITY &&

            c.j_first < c.n_out &&
            c.j_step < c.n_out &&

            (c.mac_count == 0 || c.i_first < c.n_in) &&

            !overlap(wb, we, xb, xe) &&
            !overlap(wb, we, sb, se) &&
            !overlap(xb, xe, sb, se);
    endfunction

    // --------------------------------------------------------
    // Универсальная memory используется как локальная память.
    // --------------------------------------------------------

    memory #(
        .MEM_WORDS(MEM_WORDS)
    ) u_local_memory (
        .clk_i  (clk_i),
        .rst_n_i(rst_n_i),

        .a_req_data_i (a_req),
        .a_req_valid_i(a_valid),
        .a_req_ready_o(a_ready),

        .a_rsp_data_o (a_rsp),
        .a_rsp_valid_o(a_rsp_valid),
        .a_rsp_ready_i(1'b1),

        .b_req_data_i (b_req),
        .b_req_valid_i(b_valid),
        .b_req_ready_o(b_ready),

        .b_rsp_data_o (b_rsp),
        .b_rsp_valid_o(b_rsp_valid),
        .b_rsp_ready_i(1'b1)
    );

    // --------------------------------------------------------
    // Генератор адресов.
    // --------------------------------------------------------

    pe_agu u_agu (
        .clk_i  (clk_i),
        .rst_n_i(rst_n_i),

        .cfg_i    (cfg_reg),
        .load_i   (agu_load),
        .advance_i(agu_advance),

        .agu_to_pe_data_o (agu_data),
        .agu_to_pe_valid_o(agu_valid)
    );

    // --------------------------------------------------------
    // FP32-умножитель.
    // --------------------------------------------------------

    fp32_mul_ip u_mul (
        .aclk                 (clk_i),
        .aresetn              (rst_n_i),

        .s_axis_a_tvalid      (mul_launch),
        .s_axis_a_tdata       (a_rsp.value),

        .s_axis_b_tvalid      (mul_launch),
        .s_axis_b_tdata       (b_rsp.value),

        .m_axis_result_tvalid (mul_out_valid),
        .m_axis_result_tdata  (mul_result)
    );

    // --------------------------------------------------------
    // FP32-сумматор: old_sum + product.
    // --------------------------------------------------------

    fp32_add_ip u_add (
        .aclk                 (clk_i),
        .aresetn              (rst_n_i),

        .s_axis_a_tvalid      (add_launch),
        .s_axis_a_tdata       (old_sum),

        .s_axis_b_tvalid      (add_launch),
        .s_axis_b_tdata       (product),

        .m_axis_result_tvalid (add_out_valid),
        .m_axis_result_tdata  (add_result)
    );

    // Пауза после снятия сброса для арифметических IP.
    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i)
            reset_guard <= 3'b000;
        else
            reset_guard <= {reset_guard[1:0], 1'b1};
    end

    // --------------------------------------------------------
    // Комбинационная логика управления.
    // --------------------------------------------------------

    always_comb begin
        seq_to_pe_ready_o =
            (&reset_guard) &&
            (state == CONTROL || state == HOLD);

        if (
            seq_to_pe_data_i.op == PE_WRITE &&
            state == CONTROL
        ) begin
            seq_to_pe_ready_o =
                seq_to_pe_ready_o && a_ready;
        end

        command_fire =
            seq_to_pe_valid_i && seq_to_pe_ready_o;

        cfg_legal =
            config_ok(seq_to_pe_data_i.cfg);

        write_addr_ok =
            physical_addr_ok(seq_to_pe_data_i.addr);

        agu_addr_ok =
            agu_valid &&

            physical_addr_ok(agu_data.w_addr) &&
            physical_addr_ok(agu_data.x_addr) &&
            physical_addr_ok(agu_data.s_addr) &&

            agu_data.x_addr >= cfg_reg.x_base &&
            agu_data.x_addr < x_limit &&

            agu_data.s_addr >= cfg_reg.s_base &&
            agu_data.s_addr < s_limit;

        // Начальное состояние AGU загружается при START.
        agu_load =
            command_fire &&
            seq_to_pe_data_i.op == PE_START &&
            configured;

        // Адреса продвигаются после подтверждения записи S[j].
        agu_advance =
            state == STORE_WAIT &&
            a_rsp_valid &&
            !a_rsp.error;

        // Умножение запускается по готовности W и X.
        mul_launch =
            state == PAIR_WAIT &&
            a_rsp_valid &&
            b_rsp_valid &&
            !a_rsp.error &&
            !b_rsp.error &&
            a_rsp.value[30:23] != 8'hFF &&
            b_rsp.value[30:23] != 8'hFF &&
            a_ready;

        add_launch = (state == ADD_REQ);

        // Значения по умолчанию предотвращают защёлки.
        a_req = '0;
        b_req = '0;

        a_valid = 1'b0;
        b_valid = 1'b0;

        case (state)

            CONTROL: begin
                if (
                    command_fire &&
                    seq_to_pe_data_i.op == PE_WRITE &&
                    write_addr_ok
                ) begin
                    a_req.write = 1'b1;
                    a_req.addr  = seq_to_pe_data_i.addr;
                    a_req.value = seq_to_pe_data_i.value;

                    a_valid = 1'b1;
                end
            end

            CLEAR_SUMS: begin
                a_req.write = 1'b1;
                a_req.addr  = clear_addr;
                a_req.value = 32'd0;

                a_valid = 1'b1;
            end

            MAC_READ: begin
                if (agu_addr_ok) begin
                    // Одновременно читаем W[q] и X[i].
                    a_req.addr = agu_data.w_addr;
                    b_req.addr = agu_data.x_addr;

                    a_valid = 1'b1;
                    b_valid = 1'b1;
                end
            end

            PAIR_WAIT: begin
                if (mul_launch) begin
                    // Во время запуска умножения читаем S[j].
                    a_req.addr = agu_data.s_addr;
                    a_valid = 1'b1;
                end
            end

            STORE: begin
                a_req.write = 1'b1;
                a_req.addr  = agu_data.s_addr;
                a_req.value = new_sum;

                a_valid = 1'b1;
            end

            OUT_READ: begin
                a_req.addr = out_addr;
                a_valid = 1'b1;
            end

            default: ;

        endcase

        // Выходной регистр частичной суммы.
        pe_to_reduce_data_o.neuron_id = out_neuron;
        pe_to_reduce_data_o.value     = output_value;

        pe_to_reduce_valid_o = (state == OUT_VALID);

        pe_done_o   = done_reg;
        pe_status_o = status_reg;
    end

    // --------------------------------------------------------
    // Последовательная логика.
    // --------------------------------------------------------

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            state      <= CONTROL;
            cfg_reg    <= '0;
            configured <= 1'b0;

            done_reg   <= 1'b0;
            status_reg <= PE_OK;

            clear_addr <= 0;
            out_addr   <= 0;

            x_limit <= 0;
            s_limit <= 0;

            last_neuron <= 0;
            clear_index <= 0;
            out_neuron  <= 0;

            output_value <= 0;
            old_sum      <= 0;
            product      <= 0;
            new_sum      <= 0;

            sum_seen     <= 1'b0;
            product_seen <= 1'b0;

        end else begin
            case (state)

                CONTROL, HOLD: begin
                    if (command_fire) begin
                        case (seq_to_pe_data_i.op)

                            PE_CONFIG: begin
                                if (cfg_legal) begin
                                    cfg_reg <= seq_to_pe_data_i.cfg;
                                    configured <= 1'b1;

                                    x_limit <=
                                        seq_to_pe_data_i.cfg.x_base +
                                        (
                                            {16'd0,
                                             seq_to_pe_data_i.cfg.n_in}
                                            << 2
                                        );

                                    s_limit <=
                                        seq_to_pe_data_i.cfg.s_base +
                                        (
                                            {16'd0,
                                             seq_to_pe_data_i.cfg.n_out}
                                            << 2
                                        );

                                    last_neuron <=
                                        seq_to_pe_data_i.cfg.n_out
                                        - 16'd1;

                                    done_reg <= 1'b0;
                                    status_reg <= PE_OK;

                                    state <= CONTROL;

                                end else begin
                                    status_reg <= PE_BAD_CFG;
                                    done_reg <= 1'b0;
                                    state <= ERROR;
                                end
                            end

                            PE_WRITE: begin
                                if (state != CONTROL) begin
                                    status_reg <= PE_BAD_CMD;
                                    done_reg <= 1'b0;
                                    state <= ERROR;

                                end else if (!write_addr_ok) begin
                                    status_reg <= PE_BAD_ADDR;
                                    done_reg <= 1'b0;
                                    state <= ERROR;
                                end

                                // При корректном адресе запись
                                // выполняет комбинационная логика
                                // через порт A памяти.
                            end

                            PE_START: begin
                                if (configured) begin
                                    clear_index <= 0;
                                    clear_addr <= cfg_reg.s_base;

                                    out_neuron <= 0;
                                    out_addr <= cfg_reg.s_base;

                                    done_reg <= 1'b0;

                                    sum_seen <= 1'b0;
                                    product_seen <= 1'b0;

                                    status_reg <= PE_OK;
                                    state <= CLEAR_SUMS;

                                end else begin
                                    status_reg <= PE_BAD_CMD;
                                    state <= ERROR;
                                end
                            end

                            PE_NEXT: begin
                                if (
                                    state == HOLD &&
                                    done_reg &&
                                    out_neuron < last_neuron
                                ) begin
                                    out_neuron <=
                                        out_neuron + 16'd1;

                                    out_addr <= out_addr + 32'd4;

                                    state <= OUT_READ;

                                end else begin
                                    status_reg <= PE_BAD_CMD;
                                    done_reg <= 1'b0;
                                    state <= ERROR;
                                end
                            end

                            default: begin
                                status_reg <= PE_BAD_CMD;
                                done_reg <= 1'b0;
                                state <= ERROR;
                            end

                        endcase
                    end
                end

                CLEAR_SUMS: begin
                    if (a_valid && a_ready) begin
                        if (clear_index == last_neuron) begin
                            state <= (cfg_reg.mac_count == 0)
                                ? OUT_READ
                                : MAC_READ;

                        end else begin
                            clear_index <= clear_index + 16'd1;
                            clear_addr <= clear_addr + 32'd4;
                        end
                    end
                end

                MAC_READ: begin
                    if (!agu_addr_ok) begin
                        status_reg <= PE_BAD_ADDR;
                        state <= ERROR;

                    end else if (
                        a_valid && a_ready &&
                        b_valid && b_ready
                    ) begin
                        sum_seen <= 1'b0;
                        product_seen <= 1'b0;

                        state <= PAIR_WAIT;
                    end
                end

                PAIR_WAIT: begin
                    if (a_rsp_valid && b_rsp_valid) begin
                        if (a_rsp.error || b_rsp.error) begin
                            status_reg <= PE_BAD_ADDR;
                            state <= ERROR;

                        end else if (
                            a_rsp.value[30:23] == 8'hFF ||
                            b_rsp.value[30:23] == 8'hFF
                        ) begin
                            status_reg <= PE_BAD_FP;
                            state <= ERROR;

                        end else if (mul_launch) begin
                            state <= MUL_WAIT;
                        end
                    end
                end

                MUL_WAIT: begin
                    // Ответ памяти и результат умножения
                    // могут прийти на разных тактах.
                    if (a_rsp_valid) begin
                        old_sum <= a_rsp.value;
                        sum_seen <= 1'b1;
                    end

                    if (mul_out_valid) begin
                        product <= mul_result;
                        product_seen <= 1'b1;
                    end

                    if (a_rsp_valid && a_rsp.error) begin
                        status_reg <= PE_BAD_ADDR;
                        state <= ERROR;

                    end else if (
                        (
                            a_rsp_valid &&
                            a_rsp.value[30:23] == 8'hFF
                        ) ||
                        (
                            mul_out_valid &&
                            mul_result[30:23] == 8'hFF
                        )
                    ) begin
                        status_reg <= PE_BAD_FP;
                        state <= ERROR;

                    end else if (
                        (sum_seen || a_rsp_valid) &&
                        (product_seen || mul_out_valid)
                    ) begin
                        state <= ADD_REQ;
                    end
                end

                ADD_REQ: begin
                    state <= ADD_WAIT;
                end

                ADD_WAIT: begin
                    if (add_out_valid) begin
                        if (add_result[30:23] == 8'hFF) begin
                            status_reg <= PE_BAD_FP;
                            state <= ERROR;

                        end else begin
                            new_sum <= add_result;
                            state <= STORE;
                        end
                    end
                end

                STORE: begin
                    if (a_valid && a_ready)
                        state <= STORE_WAIT;
                end

                STORE_WAIT: begin
                    if (a_rsp_valid) begin
                        if (a_rsp.error) begin
                            status_reg <= PE_BAD_ADDR;
                            state <= ERROR;

                        end else begin
                            state <= agu_data.last
                                ? OUT_READ
                                : MAC_READ;
                        end
                    end
                end

                OUT_READ: begin
                    if (a_valid && a_ready)
                        state <= OUT_WAIT;
                end

                OUT_WAIT: begin
                    if (a_rsp_valid) begin
                        if (a_rsp.error) begin
                            status_reg <= PE_BAD_ADDR;
                            done_reg <= 1'b0;
                            state <= ERROR;

                        end else if (
                            a_rsp.value[30:23] == 8'hFF
                        ) begin
                            status_reg <= PE_BAD_FP;
                            done_reg <= 1'b0;
                            state <= ERROR;

                        end else begin
                            output_value <= a_rsp.value;

                            done_reg <= 1'b1;
                            state <= OUT_VALID;
                        end
                    end
                end

                OUT_VALID: begin
                    // До handshake данные остаются
                    // в выходном регистре без изменений.
                    if (
                        pe_to_reduce_valid_o &&
                        pe_to_reduce_ready_i
                    ) begin
                        state <= HOLD;
                    end
                end

                ERROR: begin
                    // Выход из ошибки - через rst_n_i.
                    done_reg <= 1'b0;
                end

                default: begin
                    status_reg <= PE_BAD_CMD;
                    done_reg <= 1'b0;
                    state <= ERROR;
                end

            endcase
        end
    end

endmodule