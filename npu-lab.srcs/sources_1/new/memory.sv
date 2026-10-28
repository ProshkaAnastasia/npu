`timescale 1ns/1ps

module memory #(
    parameter int MEM_WORDS = 2048
) (
    input logic clk_i,
    input logic rst_n_i,

    // Порт A: клиент -> память, запрос.
    input memory_types_pkg::mem_req_t
        a_client_to_mem_request_data_i,

    input logic a_client_to_mem_request_valid_i,
    output logic a_mem_to_client_request_ready_o,

    // Порт A: память -> клиент, ответ.
    output memory_types_pkg::mem_rsp_t
        a_mem_to_client_response_data_o,

    output logic a_mem_to_client_response_valid_o,
    input  logic a_client_to_mem_response_ready_i,

    // Порт B: клиент -> память, запрос.
    input memory_types_pkg::mem_req_t
        b_client_to_mem_request_data_i,

    input logic b_client_to_mem_request_valid_i,
    output logic b_mem_to_client_request_ready_o,

    // Порт B: память -> клиент, ответ.
    output memory_types_pkg::mem_rsp_t
        b_mem_to_client_response_data_o,

    output logic b_mem_to_client_response_valid_o,
    input  logic b_client_to_mem_response_ready_i
);

    localparam int WORD_INDEX_WIDTH =
        (MEM_WORDS > 1) ? $clog2(MEM_WORDS) : 1;

    localparam logic [31:0] MEM_CAPACITY_BYTES =
        $unsigned(MEM_WORDS) << 2;

    // Каждый экземпляр memory содержит собственный массив.
    // Содержимое массива при reset не очищается.
    logic [31:0] memory_words [0:MEM_WORDS-1];

    // Данные последнего выполненного чтения.
    logic [31:0] port_a_read_value_reg;
    logic [31:0] port_b_read_value_reg;

    // Ошибка адреса для текущего ответа.
    logic port_a_error_reg;
    logic port_b_error_reg;

    // Запрос принят на текущем фронте.
    logic port_a_request_accepted;
    logic port_b_request_accepted;

    // Адрес выровнен и находится в пределах памяти.
    logic port_a_address_valid;
    logic port_b_address_valid;

    // Индексы слов, полученные из байтовых адресов.
    logic [WORD_INDEX_WIDTH-1:0] port_a_word_index;
    logic [WORD_INDEX_WIDTH-1:0] port_b_word_index;

    assign port_a_word_index =
        WORD_INDEX_WIDTH'(
            a_client_to_mem_request_data_i.addr >> 2
        );

    assign port_b_word_index =
        WORD_INDEX_WIDTH'(
            b_client_to_mem_request_data_i.addr >> 2
        );

    assign port_a_address_valid =
        a_client_to_mem_request_data_i.addr[1:0] == 2'b00 &&
        a_client_to_mem_request_data_i.addr <
            MEM_CAPACITY_BYTES;

    assign port_b_address_valid =
        b_client_to_mem_request_data_i.addr[1:0] == 2'b00 &&
        b_client_to_mem_request_data_i.addr <
            MEM_CAPACITY_BYTES;

    // Новый запрос разрешён, если предыдущего ответа нет
    // либо клиент принимает предыдущий ответ сейчас.
    assign a_mem_to_client_request_ready_o =
        !a_mem_to_client_response_valid_o ||
        a_client_to_mem_response_ready_i;

    assign b_mem_to_client_request_ready_o =
        !b_mem_to_client_response_valid_o ||
        b_client_to_mem_response_ready_i;

    assign port_a_request_accepted =
        rst_n_i &&
        a_client_to_mem_request_valid_i &&
        a_mem_to_client_request_ready_o;

    assign port_b_request_accepted =
        rst_n_i &&
        b_client_to_mem_request_valid_i &&
        b_mem_to_client_request_ready_o;

    assign a_mem_to_client_response_data_o.value =
        port_a_read_value_reg;

    assign a_mem_to_client_response_data_o.error =
        port_a_error_reg;

    assign b_mem_to_client_response_data_o.value =
        port_b_read_value_reg;

    assign b_mem_to_client_response_data_o.error =
        port_b_error_reg;

    // Порт A: синхронное чтение или запись.
    always @(posedge clk_i) begin
        if (
            port_a_request_accepted &&
            port_a_address_valid
        ) begin
            if (a_client_to_mem_request_data_i.write) begin
                memory_words[port_a_word_index] <=
                    a_client_to_mem_request_data_i.value;
            end else begin
                port_a_read_value_reg <=
                    memory_words[port_a_word_index];
            end
        end
    end

    // Порт B: синхронное чтение или запись.
    always @(posedge clk_i) begin
        if (
            port_b_request_accepted &&
            port_b_address_valid
        ) begin
            if (b_client_to_mem_request_data_i.write) begin
                memory_words[port_b_word_index] <=
                    b_client_to_mem_request_data_i.value;
            end else begin
                port_b_read_value_reg <=
                    memory_words[port_b_word_index];
            end
        end
    end

    // Формирование ответов и удержание их до принятия.
    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            a_mem_to_client_response_valid_o <= 1'b0;
            b_mem_to_client_response_valid_o <= 1'b0;

            port_a_error_reg <= 1'b0;
            port_b_error_reg <= 1'b0;

        end else begin
            if (
                a_mem_to_client_response_valid_o &&
                a_client_to_mem_response_ready_i
            )
                a_mem_to_client_response_valid_o <= 1'b0;

            if (
                b_mem_to_client_response_valid_o &&
                b_client_to_mem_response_ready_i
            )
                b_mem_to_client_response_valid_o <= 1'b0;

            // Новый запрос имеет приоритет:
            // предыдущий ответ может быть принят,
            // а новый сформирован на том же фронте.
            if (port_a_request_accepted) begin
                a_mem_to_client_response_valid_o <= 1'b1;
                port_a_error_reg <= !port_a_address_valid;
            end

            if (port_b_request_accepted) begin
                b_mem_to_client_response_valid_o <= 1'b1;
                port_b_error_reg <= !port_b_address_valid;
            end
        end
    end

endmodule