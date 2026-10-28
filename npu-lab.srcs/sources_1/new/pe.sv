`timescale 1ns/1ps

module pe #(
    // Количество 32-битных слов локальной памяти.
    // 2048 слов = 8 КиБ.
    parameter int MEM_WORDS = 2048
) (
    input logic clk_i,
    input logic rst_n_i,

    // Секвенсор -> PE: команда и её параметры.
    input pe_types_pkg::seq_to_pe_t seq_to_pe_command_i,
    input logic seq_to_pe_command_valid_i,

    // PE -> секвенсор: готовность принять команду.
    output logic pe_to_seq_command_ready_o,

    // PE -> reduce: частичная сумма текущего нейрона.
    output logic [31:0] pe_to_reduce_sum_o,

    // PE -> секвенсор: текущая сумма выставлена на выходе.
    output logic pe_to_seq_sum_valid_o,

    // PE -> секвенсор: все суммы слоя вычислены.
    // Впервые устанавливается после выставления первой суммы.
    output logic pe_to_seq_layer_done_o,

    // PE -> секвенсор: статус выполнения.
    output pe_types_pkg::pe_status_t pe_to_seq_status_o
);

    import pe_types_pkg::*;
    import memory_types_pkg::*;

    localparam logic [31:0] LOCAL_MEM_BYTES =
        $unsigned(MEM_WORDS) << 2;

    localparam logic [31:0] LOCAL_MEM_WORDS =
        $unsigned(MEM_WORDS);

    localparam logic [63:0] LOCAL_MEM_CAPACITY_BYTES =
        {32'd0, LOCAL_MEM_BYTES};

    typedef enum logic [3:0] {
        WAIT_SEQ_COMMAND,
        CLEAR_LOCAL_SUMS,
        REQUEST_WEIGHT_AND_INPUT,
        WAIT_WEIGHT_AND_INPUT,
        WAIT_PRODUCT_AND_PREVIOUS_SUM,
        ISSUE_ACCUMULATE,
        WAIT_ACCUMULATE_RESULT,
        WRITE_UPDATED_SUM,
        WAIT_SUM_WRITE_RESPONSE,
        REQUEST_OUTPUT_SUM,
        WAIT_OUTPUT_SUM,
        HOLD_OUTPUT_SUM,
        FAULT
    } state_t;

    state_t pe_state;

    // Конфигурация текущего слоя.
    pe_cfg_t layer_config_reg;

    pe_status_t error_status_reg;

    logic config_valid_reg;
    logic layer_done_reg;

    logic [2:0] reset_release_pipe;

    // Приём и проверка команды секвенсора.
    logic seq_command_accepted;
    logic seq_write_address_valid;
    logic seq_config_valid;

    // Проверка адресов, выданных AGU.
    logic agu_addresses_valid;

    // Адрес обнуления очередной локальной суммы.
    logic [31:0] sum_clear_address_reg;

    // Адрес суммы, которую нужно выставить на выход.
    logic [31:0] sum_output_address_reg;

    // Исключающие верхние границы областей X и S.
    logic [31:0] input_region_end_reg;
    logic [31:0] sum_region_end_reg;

    logic [15:0] last_neuron_index_reg;
    logic [15:0] sum_clear_index_reg;
    logic [15:0] output_neuron_index_reg;

    // Выходной регистр частичной суммы.
    logic [31:0] output_sum_reg;

    // Регистры вычисления S[j] + W[q] * X[i].
    logic [31:0] previous_sum_reg;
    logic [31:0] weight_input_product_reg;
    logic [31:0] updated_sum_reg;

    logic previous_sum_received_reg;
    logic product_received_reg;

    // PE -> локальная память: запросы портов A и B.
    mem_req_t pe_to_local_mem_a_request;
    mem_req_t pe_to_local_mem_b_request;

    logic pe_to_local_mem_a_request_valid;
    logic pe_to_local_mem_b_request_valid;

    // Локальная память -> PE: готовность принять запросы.
    logic local_mem_a_to_pe_request_ready;
    logic local_mem_b_to_pe_request_ready;

    // Локальная память -> PE: ответы на запросы.
    mem_rsp_t local_mem_a_to_pe_response;
    mem_rsp_t local_mem_b_to_pe_response;

    logic local_mem_a_to_pe_response_valid;
    logic local_mem_b_to_pe_response_valid;

    // AGU -> PE: адреса текущей операции MAC.
    agu_to_pe_t agu_to_pe_addresses;
    logic agu_to_pe_addresses_valid;

    // PE -> AGU: загрузка начального состояния
    // и переход к следующей операции MAC.
    logic pe_to_agu_load;
    logic pe_to_agu_advance;

    // PE -> умножитель: запуск произведения W * X.
    logic pe_to_multiplier_request_valid;

    // Умножитель -> PE: результат произведения.
    logic [31:0] multiplier_to_pe_result;
    logic multiplier_to_pe_result_valid;

    // PE -> сумматор: запуск S + произведение.
    logic pe_to_adder_request_valid;

    // Сумматор -> PE: обновлённая сумма.
    logic [31:0] adder_to_pe_result;
    logic adder_to_pe_result_valid;

    function automatic logic local_address_valid(
        input logic [31:0] addr
    );
        return
            addr[1:0] == 2'b00 &&
            addr < LOCAL_MEM_BYTES;
    endfunction

    function automatic logic address_ranges_overlap(
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

    // Проверка размещения активных областей памяти.
    // Соответствие mac_count и шагов распределению весов
    // обеспечивает секвенсор.
    function automatic logic layer_configuration_valid(
        input pe_cfg_t candidate_config
    );
        logic [63:0] weights_begin;
        logic [63:0] weights_end;

        logic [63:0] inputs_begin;
        logic [63:0] inputs_end;

        logic [63:0] sums_begin;
        logic [63:0] sums_end;

        weights_begin = {
            32'd0,
            candidate_config.w_base
        };

        inputs_begin = {
            32'd0,
            candidate_config.x_base
        };

        sums_begin = {
            32'd0,
            candidate_config.s_base
        };

        weights_end = weights_begin +
            ({32'd0, candidate_config.mac_count} << 2);

        inputs_end = inputs_begin +
            ({48'd0, candidate_config.n_in} << 2);

        sums_end = sums_begin +
            ({48'd0, candidate_config.n_out} << 2);

        return
            candidate_config.n_in != 0 &&
            candidate_config.n_out != 0 &&

            candidate_config.w_base[1:0] == 2'b00 &&
            candidate_config.x_base[1:0] == 2'b00 &&
            candidate_config.s_base[1:0] == 2'b00 &&

            candidate_config.mac_count <= LOCAL_MEM_WORDS &&

            weights_end <= LOCAL_MEM_CAPACITY_BYTES &&
            inputs_end <= LOCAL_MEM_CAPACITY_BYTES &&
            sums_end <= LOCAL_MEM_CAPACITY_BYTES &&

            candidate_config.j_first <
                candidate_config.n_out &&

            candidate_config.j_step <
                candidate_config.n_out &&

            (
                candidate_config.mac_count == 0 ||
                candidate_config.i_first <
                    candidate_config.n_in
            ) &&

            !address_ranges_overlap(
                weights_begin,
                weights_end,
                inputs_begin,
                inputs_end
            ) &&

            !address_ranges_overlap(
                weights_begin,
                weights_end,
                sums_begin,
                sums_end
            ) &&

            !address_ranges_overlap(
                inputs_begin,
                inputs_end,
                sums_begin,
                sums_end
            );
    endfunction

        memory #(
        .MEM_WORDS(MEM_WORDS)
    ) u_local_memory (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),

        // Порт A: запрос PE -> локальная память.
        .a_client_to_mem_request_data_i(
            pe_to_local_mem_a_request
        ),
        .a_client_to_mem_request_valid_i(
            pe_to_local_mem_a_request_valid
        ),
        .a_mem_to_client_request_ready_o(
            local_mem_a_to_pe_request_ready
        ),

        // Порт A: ответ локальной памяти -> PE.
        .a_mem_to_client_response_data_o(
            local_mem_a_to_pe_response
        ),
        .a_mem_to_client_response_valid_o(
            local_mem_a_to_pe_response_valid
        ),
        .a_client_to_mem_response_ready_i(1'b1),

        // Порт B: запрос PE -> локальная память.
        .b_client_to_mem_request_data_i(
            pe_to_local_mem_b_request
        ),
        .b_client_to_mem_request_valid_i(
            pe_to_local_mem_b_request_valid
        ),
        .b_mem_to_client_request_ready_o(
            local_mem_b_to_pe_request_ready
        ),

        // Порт B: ответ локальной памяти -> PE.
        .b_mem_to_client_response_data_o(
            local_mem_b_to_pe_response
        ),
        .b_mem_to_client_response_valid_o(
            local_mem_b_to_pe_response_valid
        ),
        .b_client_to_mem_response_ready_i(1'b1)
    );

    pe_agu u_address_generator (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),

        .cfg_i(layer_config_reg),
        .load_i(pe_to_agu_load),
        .advance_i(pe_to_agu_advance),

        .agu_to_pe_data_o(agu_to_pe_addresses),
        .agu_to_pe_valid_o(agu_to_pe_addresses_valid)
    );

    // Во время чтения пары:
    // порт A возвращает вес, порт B - входное значение.
    fp32_mul_ip u_multiplier (
        .aclk(clk_i),
        .aresetn(rst_n_i),

        .s_axis_a_tvalid(pe_to_multiplier_request_valid),
        .s_axis_a_tdata(local_mem_a_to_pe_response.value),

        .s_axis_b_tvalid(pe_to_multiplier_request_valid),
        .s_axis_b_tdata(local_mem_b_to_pe_response.value),

        .m_axis_result_tvalid(multiplier_to_pe_result_valid),
        .m_axis_result_tdata(multiplier_to_pe_result)
    );

    fp32_add_ip u_adder (
        .aclk(clk_i),
        .aresetn(rst_n_i),

        .s_axis_a_tvalid(pe_to_adder_request_valid),
        .s_axis_a_tdata(previous_sum_reg),

        .s_axis_b_tvalid(pe_to_adder_request_valid),
        .s_axis_b_tdata(weight_input_product_reg),

        .m_axis_result_tvalid(adder_to_pe_result_valid),
        .m_axis_result_tdata(adder_to_pe_result)
    );

    // Защитная пауза после снятия сброса.
    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i)
            reset_release_pipe <= '0;
        else
            reset_release_pipe <= {
                reset_release_pipe[1:0],
                1'b1
            };
    end

    always_comb begin
        pe_to_seq_command_ready_o =
            (&reset_release_pipe) &&
            (
                pe_state == WAIT_SEQ_COMMAND ||
                pe_state == HOLD_OUTPUT_SUM
            );

        if (
            seq_to_pe_command_i.op == PE_WRITE &&
            pe_state == WAIT_SEQ_COMMAND
        ) begin
            pe_to_seq_command_ready_o =
                pe_to_seq_command_ready_o &&
                local_mem_a_to_pe_request_ready;
        end

        seq_command_accepted =
            seq_to_pe_command_valid_i &&
            pe_to_seq_command_ready_o;

        seq_config_valid =
            layer_configuration_valid(
                seq_to_pe_command_i.cfg
            );

        seq_write_address_valid =
            local_address_valid(
                seq_to_pe_command_i.addr
            );

        agu_addresses_valid =
            agu_to_pe_addresses_valid &&

            local_address_valid(
                agu_to_pe_addresses.w_addr
            ) &&

            local_address_valid(
                agu_to_pe_addresses.x_addr
            ) &&

            local_address_valid(
                agu_to_pe_addresses.s_addr
            ) &&

            agu_to_pe_addresses.x_addr >=
                layer_config_reg.x_base &&

            agu_to_pe_addresses.x_addr <
                input_region_end_reg &&

            agu_to_pe_addresses.s_addr >=
                layer_config_reg.s_base &&

            agu_to_pe_addresses.s_addr <
                sum_region_end_reg;

        // Начальное состояние AGU загружается при START.
        pe_to_agu_load =
            seq_command_accepted &&
            seq_to_pe_command_i.op == PE_START &&
            config_valid_reg;

        // Адреса продвигаются после подтверждения записи S[j].
        pe_to_agu_advance =
            pe_state == WAIT_SUM_WRITE_RESPONSE &&
            local_mem_a_to_pe_response_valid &&
            !local_mem_a_to_pe_response.error;

        // Запуск произведения после получения W и X.
        // Одновременно порт A должен принять чтение S[j].
        pe_to_multiplier_request_valid =
            pe_state == WAIT_WEIGHT_AND_INPUT &&

            local_mem_a_to_pe_response_valid &&
            local_mem_b_to_pe_response_valid &&

            !local_mem_a_to_pe_response.error &&
            !local_mem_b_to_pe_response.error &&

            local_mem_a_to_pe_response.value[30:23] !=
                8'hFF &&

            local_mem_b_to_pe_response.value[30:23] !=
                8'hFF &&

            local_mem_a_to_pe_request_ready;

        pe_to_adder_request_valid =
            (pe_state == ISSUE_ACCUMULATE);

        pe_to_local_mem_a_request = '0;
        pe_to_local_mem_b_request = '0;

        pe_to_local_mem_a_request_valid = 1'b0;
        pe_to_local_mem_b_request_valid = 1'b0;

        case (pe_state)

            WAIT_SEQ_COMMAND: begin
                if (
                    seq_command_accepted &&
                    seq_to_pe_command_i.op == PE_WRITE &&
                    seq_write_address_valid
                ) begin
                    pe_to_local_mem_a_request.write = 1'b1;

                    pe_to_local_mem_a_request.addr =
                        seq_to_pe_command_i.addr;

                    pe_to_local_mem_a_request.value =
                        seq_to_pe_command_i.value;

                    pe_to_local_mem_a_request_valid = 1'b1;
                end
            end

            CLEAR_LOCAL_SUMS: begin
                pe_to_local_mem_a_request.write = 1'b1;

                pe_to_local_mem_a_request.addr =
                    sum_clear_address_reg;

                pe_to_local_mem_a_request.value = 32'd0;

                pe_to_local_mem_a_request_valid = 1'b1;
            end

            REQUEST_WEIGHT_AND_INPUT: begin
                if (agu_addresses_valid) begin
                    pe_to_local_mem_a_request.addr =
                        agu_to_pe_addresses.w_addr;

                    pe_to_local_mem_b_request.addr =
                        agu_to_pe_addresses.x_addr;

                    pe_to_local_mem_a_request_valid = 1'b1;
                    pe_to_local_mem_b_request_valid = 1'b1;
                end
            end

            WAIT_WEIGHT_AND_INPUT: begin
                if (pe_to_multiplier_request_valid) begin
                    pe_to_local_mem_a_request.addr =
                        agu_to_pe_addresses.s_addr;

                    pe_to_local_mem_a_request_valid = 1'b1;
                end
            end

            WRITE_UPDATED_SUM: begin
                pe_to_local_mem_a_request.write = 1'b1;

                pe_to_local_mem_a_request.addr =
                    agu_to_pe_addresses.s_addr;

                pe_to_local_mem_a_request.value =
                    updated_sum_reg;

                pe_to_local_mem_a_request_valid = 1'b1;
            end

            REQUEST_OUTPUT_SUM: begin
                pe_to_local_mem_a_request.addr =
                    sum_output_address_reg;

                pe_to_local_mem_a_request_valid = 1'b1;
            end

            default: ;

        endcase

        pe_to_reduce_sum_o = output_sum_reg;

        pe_to_seq_sum_valid_o =
            (pe_state == HOLD_OUTPUT_SUM);

        pe_to_seq_layer_done_o = layer_done_reg;
        pe_to_seq_status_o = error_status_reg;
    end

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            pe_state <= WAIT_SEQ_COMMAND;

            layer_config_reg <= '0;
            config_valid_reg <= 1'b0;

            layer_done_reg <= 1'b0;
            error_status_reg <= PE_OK;

            sum_clear_address_reg <= '0;
            sum_output_address_reg <= '0;

            input_region_end_reg <= '0;
            sum_region_end_reg <= '0;

            last_neuron_index_reg <= '0;
            sum_clear_index_reg <= '0;
            output_neuron_index_reg <= '0;

            output_sum_reg <= '0;
            previous_sum_reg <= '0;
            weight_input_product_reg <= '0;
            updated_sum_reg <= '0;

            previous_sum_received_reg <= 1'b0;
            product_received_reg <= 1'b0;

        end else begin
            case (pe_state)

                WAIT_SEQ_COMMAND,
                HOLD_OUTPUT_SUM: begin
                    if (seq_command_accepted) begin
                        case (seq_to_pe_command_i.op)

                            PE_CONFIG: begin
                                if (seq_config_valid) begin
                                    layer_config_reg <=
                                        seq_to_pe_command_i.cfg;

                                    config_valid_reg <= 1'b1;

                                    input_region_end_reg <=
                                        seq_to_pe_command_i.cfg.x_base +
                                        (
                                            {
                                                16'd0,
                                                seq_to_pe_command_i.cfg.n_in
                                            } << 2
                                        );

                                    sum_region_end_reg <=
                                        seq_to_pe_command_i.cfg.s_base +
                                        (
                                            {
                                                16'd0,
                                                seq_to_pe_command_i.cfg.n_out
                                            } << 2
                                        );

                                    last_neuron_index_reg <=
                                        seq_to_pe_command_i.cfg.n_out -
                                        16'd1;

                                    layer_done_reg <= 1'b0;
                                    error_status_reg <= PE_OK;

                                    pe_state <= WAIT_SEQ_COMMAND;

                                end else begin
                                    error_status_reg <= PE_BAD_CFG;
                                    layer_done_reg <= 1'b0;
                                    pe_state <= FAULT;
                                end
                            end

                            PE_WRITE: begin
                                if (
                                    pe_state != WAIT_SEQ_COMMAND
                                ) begin
                                    error_status_reg <= PE_BAD_CMD;
                                    layer_done_reg <= 1'b0;
                                    pe_state <= FAULT;

                                end else if (
                                    !seq_write_address_valid
                                ) begin
                                    error_status_reg <= PE_BAD_ADDR;
                                    layer_done_reg <= 1'b0;
                                    pe_state <= FAULT;
                                end

                                // Корректная запись выполняется
                                // через порт A локальной памяти.
                            end

                            PE_START: begin
                                if (config_valid_reg) begin
                                    sum_clear_index_reg <= '0;

                                    sum_clear_address_reg <=
                                        layer_config_reg.s_base;

                                    output_neuron_index_reg <= '0;

                                    sum_output_address_reg <=
                                        layer_config_reg.s_base;

                                    layer_done_reg <= 1'b0;

                                    previous_sum_received_reg <= 1'b0;
                                    product_received_reg <= 1'b0;

                                    error_status_reg <= PE_OK;
                                    pe_state <= CLEAR_LOCAL_SUMS;

                                end else begin
                                    error_status_reg <= PE_BAD_CMD;
                                    pe_state <= FAULT;
                                end
                            end

                            PE_NEXT: begin
                                if (
                                    pe_state == HOLD_OUTPUT_SUM &&
                                    layer_done_reg &&
                                    output_neuron_index_reg <
                                        last_neuron_index_reg
                                ) begin
                                    output_neuron_index_reg <=
                                        output_neuron_index_reg +
                                        16'd1;

                                    sum_output_address_reg <=
                                        sum_output_address_reg +
                                        32'd4;

                                    pe_state <= REQUEST_OUTPUT_SUM;

                                end else begin
                                    error_status_reg <= PE_BAD_CMD;
                                    layer_done_reg <= 1'b0;
                                    pe_state <= FAULT;
                                end
                            end

                            default: begin
                                error_status_reg <= PE_BAD_CMD;
                                layer_done_reg <= 1'b0;
                                pe_state <= FAULT;
                            end

                        endcase
                    end
                end

                CLEAR_LOCAL_SUMS: begin
                    if (
                        pe_to_local_mem_a_request_valid &&
                        local_mem_a_to_pe_request_ready
                    ) begin
                        if (
                            sum_clear_index_reg ==
                                last_neuron_index_reg
                        ) begin
                            pe_state <=
                                (layer_config_reg.mac_count == 0)
                                    ? REQUEST_OUTPUT_SUM
                                    : REQUEST_WEIGHT_AND_INPUT;

                        end else begin
                            sum_clear_index_reg <=
                                sum_clear_index_reg + 16'd1;

                            sum_clear_address_reg <=
                                sum_clear_address_reg + 32'd4;
                        end
                    end
                end

                REQUEST_WEIGHT_AND_INPUT: begin
                    if (!agu_addresses_valid) begin
                        error_status_reg <= PE_BAD_ADDR;
                        pe_state <= FAULT;

                    end else if (
                        pe_to_local_mem_a_request_valid &&
                        local_mem_a_to_pe_request_ready &&
                        pe_to_local_mem_b_request_valid &&
                        local_mem_b_to_pe_request_ready
                    ) begin
                        previous_sum_received_reg <= 1'b0;
                        product_received_reg <= 1'b0;

                        pe_state <= WAIT_WEIGHT_AND_INPUT;
                    end
                end

                WAIT_WEIGHT_AND_INPUT: begin
                    if (
                        local_mem_a_to_pe_response_valid &&
                        local_mem_b_to_pe_response_valid
                    ) begin
                        if (
                            local_mem_a_to_pe_response.error ||
                            local_mem_b_to_pe_response.error
                        ) begin
                            error_status_reg <= PE_BAD_ADDR;
                            pe_state <= FAULT;

                        end else if (
                            local_mem_a_to_pe_response.value[30:23]
                                == 8'hFF ||
                            local_mem_b_to_pe_response.value[30:23]
                                == 8'hFF
                        ) begin
                            error_status_reg <= PE_BAD_FP;
                            pe_state <= FAULT;

                        end else if (
                            pe_to_multiplier_request_valid
                        ) begin
                            pe_state <=
                                WAIT_PRODUCT_AND_PREVIOUS_SUM;
                        end
                    end
                end

                WAIT_PRODUCT_AND_PREVIOUS_SUM: begin
                    // Предыдущая сумма и произведение могут
                    // поступить на разных тактах.
                    if (local_mem_a_to_pe_response_valid) begin
                        previous_sum_reg <=
                            local_mem_a_to_pe_response.value;

                        previous_sum_received_reg <= 1'b1;
                    end

                    if (multiplier_to_pe_result_valid) begin
                        weight_input_product_reg <=
                            multiplier_to_pe_result;

                        product_received_reg <= 1'b1;
                    end

                    if (
                        local_mem_a_to_pe_response_valid &&
                        local_mem_a_to_pe_response.error
                    ) begin
                        error_status_reg <= PE_BAD_ADDR;
                        pe_state <= FAULT;

                    end else if (
                        (
                            local_mem_a_to_pe_response_valid &&
                            local_mem_a_to_pe_response.value[30:23]
                                == 8'hFF
                        ) ||
                        (
                            multiplier_to_pe_result_valid &&
                            multiplier_to_pe_result[30:23]
                                == 8'hFF
                        )
                    ) begin
                        error_status_reg <= PE_BAD_FP;
                        pe_state <= FAULT;

                    end else if (
                        (
                            previous_sum_received_reg ||
                            local_mem_a_to_pe_response_valid
                        ) &&
                        (
                            product_received_reg ||
                            multiplier_to_pe_result_valid
                        )
                    ) begin
                        pe_state <= ISSUE_ACCUMULATE;
                    end
                end

                ISSUE_ACCUMULATE: begin
                    pe_state <= WAIT_ACCUMULATE_RESULT;
                end

                WAIT_ACCUMULATE_RESULT: begin
                    if (adder_to_pe_result_valid) begin
                        if (
                            adder_to_pe_result[30:23] == 8'hFF
                        ) begin
                            error_status_reg <= PE_BAD_FP;
                            pe_state <= FAULT;

                        end else begin
                            updated_sum_reg <= adder_to_pe_result;
                            pe_state <= WRITE_UPDATED_SUM;
                        end
                    end
                end

                WRITE_UPDATED_SUM: begin
                    if (
                        pe_to_local_mem_a_request_valid &&
                        local_mem_a_to_pe_request_ready
                    )
                        pe_state <= WAIT_SUM_WRITE_RESPONSE;
                end

                WAIT_SUM_WRITE_RESPONSE: begin
                    if (local_mem_a_to_pe_response_valid) begin
                        if (local_mem_a_to_pe_response.error) begin
                            error_status_reg <= PE_BAD_ADDR;
                            pe_state <= FAULT;

                        end else begin
                            pe_state <= agu_to_pe_addresses.last
                                ? REQUEST_OUTPUT_SUM
                                : REQUEST_WEIGHT_AND_INPUT;
                        end
                    end
                end

                REQUEST_OUTPUT_SUM: begin
                    if (
                        pe_to_local_mem_a_request_valid &&
                        local_mem_a_to_pe_request_ready
                    )
                        pe_state <= WAIT_OUTPUT_SUM;
                end

                WAIT_OUTPUT_SUM: begin
                    if (local_mem_a_to_pe_response_valid) begin
                        if (local_mem_a_to_pe_response.error) begin
                            error_status_reg <= PE_BAD_ADDR;
                            layer_done_reg <= 1'b0;
                            pe_state <= FAULT;

                        end else if (
                            local_mem_a_to_pe_response.value[30:23]
                                == 8'hFF
                        ) begin
                            error_status_reg <= PE_BAD_FP;
                            layer_done_reg <= 1'b0;
                            pe_state <= FAULT;

                        end else begin
                            output_sum_reg <=
                                local_mem_a_to_pe_response.value;

                            layer_done_reg <= 1'b1;
                            pe_state <= HOLD_OUTPUT_SUM;
                        end
                    end
                end

                FAULT: begin
                    // Выход из ошибки - через reset.
                    layer_done_reg <= 1'b0;
                end

                default: begin
                    error_status_reg <= PE_BAD_CMD;
                    layer_done_reg <= 1'b0;
                    pe_state <= FAULT;
                end

            endcase
        end
    end

endmodule