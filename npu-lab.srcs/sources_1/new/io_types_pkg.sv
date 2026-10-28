`timescale 1ns/1ps

package io_types_pkg;

    // Команды UART.
    typedef enum logic [7:0] {
        CMD_CONFIG_BEGIN = 8'h01,
        CMD_CONFIG_DATA  = 8'h02,
        CMD_CONFIG_END   = 8'h03,

        CMD_INPUT_BEGIN  = 8'h04,
        CMD_INPUT_DATA   = 8'h05,
        CMD_INPUT_END    = 8'h06,

        CMD_RUN          = 8'h07,
        CMD_READ_WORD    = 8'h08,

        CMD_RESPONSE     = 8'h80
    } io_cmd_t;

    // Статусы ответов.
    typedef enum logic [7:0] {
        STATUS_OK            = 8'h00,
        STATUS_CRC_ERROR     = 8'h01,
        STATUS_FORMAT_ERROR  = 8'h02,
        STATUS_TIMEOUT       = 8'h03,
        STATUS_SEQ_ERROR     = 8'h04,

        STATUS_BAD_STATE     = 8'h10,
        STATUS_BAD_SIZE      = 8'h11,
        STATUS_BAD_OFFSET    = 8'h12,
        STATUS_BAD_STREAM    = 8'h13,
        STATUS_MODEL_CRC     = 8'h14,
        STATUS_BAD_MODEL     = 8'h15,
        STATUS_MEMORY_ERROR  = 8'h16,
        STATUS_PE_ERROR      = 8'h17,
        STATUS_COMPUTE_ERROR = 8'h18
    } io_status_t;

    // Данные от IO-контроллера секвенсору.
    //
    // Первая передача пакета:
    //   is_header = 1;
    //   действительны cmd, seq и параметры команды.
    //
    // Последующие передачи CONFIG_DATA / INPUT_DATA:
    //   is_header = 0;
    //   действительны word, keep, last.
    typedef struct packed {
        logic        is_header;

        logic [7:0]  cmd;
        logic [15:0] seq;

        // Размер образа или входного вектора в байтах.
        // Используется командами CONFIG_BEGIN / INPUT_BEGIN.
        logic [31:0] image_size;

        // Смещение в байтах для CONFIG_DATA / INPUT_DATA.
        // Для READ_WORD - байт-адрес общей памяти.
        logic [31:0] offset;

        // Количество байт данных в текущей части,
        // без четырёх байт поля offset.
        logic [15:0] data_length;

        // Слово полезной нагрузки.
        // Первый по порядку байт находится в word[7:0].
        logic [31:0] word;

        // keep[k] показывает действительность байта
        // word[8*k +: 8].
        logic [3:0]  keep;

        // Последнее слово полезной нагрузки пакета.
        logic        last;
    } io_to_seq_t;

    // Ответ секвенсора IO-контроллеру.
    typedef struct packed {
        // Номер исходного запроса.
        logic [15:0] seq;

        // Код завершения команды.
        logic [7:0] status;

        // Значение зависит от команды:
        // CONFIG_DATA / INPUT_DATA - следующее смещение;
        // CONFIG_END / INPUT_END   - принятый размер;
        // RUN                      - адрес выходного буфера;
        // READ_WORD                - прочитанные 32 бита.
        logic [31:0] next_offset;
    } seq_to_io_t;

endpackage