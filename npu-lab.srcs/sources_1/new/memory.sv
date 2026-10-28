`timescale 1ns/1ps

module memory #(
    parameter int MEM_WORDS = 2048
) (
    input logic clk_i,
    input logic rst_n_i,

    // Порт A: запрос.
    input memory_types_pkg::mem_req_t a_req_data_i,
    input  logic a_req_valid_i,
    output logic a_req_ready_o,

    // Порт A: ответ.
    output memory_types_pkg::mem_rsp_t a_rsp_data_o,
    output logic a_rsp_valid_o,
    input  logic a_rsp_ready_i,

    // Порт B: запрос.
    input memory_types_pkg::mem_req_t b_req_data_i,
    input  logic b_req_valid_i,
    output logic b_req_ready_o,

    // Порт B: ответ.
    output memory_types_pkg::mem_rsp_t b_rsp_data_o,
    output logic b_rsp_valid_o,
    input  logic b_rsp_ready_i
);

    localparam int ADDR_WIDTH =
        (MEM_WORDS > 1) ? $clog2(MEM_WORDS) : 1;

    localparam logic [31:0] MEM_BYTES =
        $unsigned(MEM_WORDS) << 2;

    // Каждый экземпляр модуля содержит свой массив.
    logic [31:0] storage [0:MEM_WORDS-1];

    logic [31:0] a_read_data;
    logic [31:0] b_read_data;

    logic a_error;
    logic b_error;

    logic a_accept;
    logic b_accept;

    logic a_addr_ok;
    logic b_addr_ok;

    logic [ADDR_WIDTH-1:0] a_word_addr;
    logic [ADDR_WIDTH-1:0] b_word_addr;

    // Байтовый адрес -> номер 32-битного слова.
    assign a_word_addr =
        ADDR_WIDTH'(a_req_data_i.addr >> 2);

    assign b_word_addr =
        ADDR_WIDTH'(b_req_data_i.addr >> 2);

    // Проверка выравнивания и границ.
    assign a_addr_ok =
        a_req_data_i.addr[1:0] == 2'b00 &&
        a_req_data_i.addr < MEM_BYTES;

    assign b_addr_ok =
        b_req_data_i.addr[1:0] == 2'b00 &&
        b_req_data_i.addr < MEM_BYTES;

    // Новый запрос разрешён, если:
    // - предыдущего ответа нет;
    // - либо предыдущий ответ принимается сейчас.
    assign a_req_ready_o =
        !a_rsp_valid_o || a_rsp_ready_i;

    assign b_req_ready_o =
        !b_rsp_valid_o || b_rsp_ready_i;

    assign a_accept =
        rst_n_i &&
        a_req_valid_i &&
        a_req_ready_o;

    assign b_accept =
        rst_n_i &&
        b_req_valid_i &&
        b_req_ready_o;

    assign a_rsp_data_o.value = a_read_data;
    assign a_rsp_data_o.error = a_error;

    assign b_rsp_data_o.value = b_read_data;
    assign b_rsp_data_o.error = b_error;

    // --------------------------------------------------------
    // Порт A: синхронное чтение и запись.
    // Содержимое памяти не сбрасывается.
    // --------------------------------------------------------

    always @(posedge clk_i) begin
        if (a_accept && a_addr_ok) begin
            if (a_req_data_i.write) begin
                storage[a_word_addr] <= a_req_data_i.value;
            end else begin
                a_read_data <= storage[a_word_addr];
            end
        end
    end

    // --------------------------------------------------------
    // Порт B: синхронное чтение и запись.
    // --------------------------------------------------------

    always @(posedge clk_i) begin
        if (b_accept && b_addr_ok) begin
            if (b_req_data_i.write) begin
                storage[b_word_addr] <= b_req_data_i.value;
            end else begin
                b_read_data <= storage[b_word_addr];
            end
        end
    end

    // --------------------------------------------------------
    // Признаки наличия ответов и ошибки.
    // --------------------------------------------------------

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            a_rsp_valid_o <= 1'b0;
            b_rsp_valid_o <= 1'b0;

            a_error <= 1'b0;
            b_error <= 1'b0;

        end else begin
            if (a_rsp_valid_o && a_rsp_ready_i)
                a_rsp_valid_o <= 1'b0;

            if (b_rsp_valid_o && b_rsp_ready_i)
                b_rsp_valid_o <= 1'b0;

            // Новое принятие имеет приоритет:
            // старый ответ может уйти, а новый появиться
            // на том же фронте.
            if (a_accept) begin
                a_rsp_valid_o <= 1'b1;
                a_error <= !a_addr_ok;
            end

            if (b_accept) begin
                b_rsp_valid_o <= 1'b1;
                b_error <= !b_addr_ok;
            end
        end
    end

endmodule