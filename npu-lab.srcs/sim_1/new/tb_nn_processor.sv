`timescale 1ns/1ps

// Проверка всей системы через физические линии UART.
module tb_nn_processor;

    import io_types_pkg::*;

    logic clk = 0;
    always #5 clk = ~clk;

    logic rst = 0;
    logic rx = 1;
    logic tx;

    logic model_valid;
    logic busy;

    // Ускоренный UART только для симуляции:
    // 100 MHz / 12.5 Mbaud = 8 тактов на бит.
    localparam time BIT_TIME = 80ns;

    integer sequence_id = 0;
    integer received = 0;
    integer checked = 0;

    logic [7:0] payload [0:259];
    logic [7:0] response_bytes [0:2047];

    logic [31:0] model [0:662];
    logic [31:0] inputs [0:48];
    logic [31:0] golden [0:14];

    nn_processor_top #(
        .UART_BAUD_RATE(12_500_000)
    ) dut (
        .clk_i(clk),
        .rst_n_i(rst),

        .uart_rx_i(rx),
        .uart_tx_o(tx),

        .model_valid_o(model_valid),
        .busy_o(busy)
    );

    function automatic logic [31:0] crc_byte(
        input logic [31:0] c0,
        input logic [7:0] b
    );
        logic [31:0] c;

        c = c0 ^ {24'd0, b};

        for (int k = 0; k < 8; k++)
            c = c[0] ? ((c >> 1) ^ 32'hEDB88320)
                     :  (c >> 1);

        return c;
    endfunction

    // Передача одного байта по UART, формат 8N1.
    task automatic send_byte(input logic [7:0] b);
        @(negedge clk);

        rx = 0;
        #(BIT_TIME);

        for (int i = 0; i < 8; i++) begin
            rx = b[i];
            #(BIT_TIME);
        end

        rx = 1;
        #(BIT_TIME);
    endtask

    // Независимый приёмник ответов процессора.
    initial begin : monitor
        logic [7:0] b;

        wait (rst);

        forever begin
            @(negedge tx);
            #(BIT_TIME / 2);

            if (tx !== 0)
                $fatal(1, "Bad start bit");

            for (int i = 0; i < 8; i++) begin
                #(BIT_TIME);
                b[i] = tx;
            end

            #(BIT_TIME);

            if (tx !== 1)
                $fatal(1, "Bad stop bit");

            response_bytes[received] = b;
            received = received + 1;

            #(BIT_TIME / 2);
        end
    end

    // Отправка пакета и проверка ответа.
    //
    // arg       - размер или смещение/адрес.
    // n         - количество передаваемых 32-битных слов.
    // index     - начальный индекс массива model / inputs.
    // use_input - брать слова из inputs вместо model.
    // bad_crc   - намеренно испортить CRC пакета.
    task automatic transaction(
        input logic [7:0] cmd,
        input logic [31:0] arg,

        input integer n,
        input integer index,

        input logic use_input,
        input logic bad_crc,

        input logic [7:0] expected_status,
        input logic [31:0] expected_value
    );
        integer len;
        integer base;

        logic [31:0] c;
        logic [31:0] w;

        sequence_id = sequence_id + 1;
        len = 0;

        if (
            cmd == CMD_CONFIG_BEGIN ||
            cmd == CMD_INPUT_BEGIN ||
            cmd == CMD_READ_WORD ||
            cmd == CMD_CONFIG_DATA ||
            cmd == CMD_INPUT_DATA
        ) begin
            for (int b = 0; b < 4; b++)
                payload[b] = arg[8*b +: 8];

            len = 4;
        end

        for (int i = 0; i < n; i++) begin
            w = use_input ? inputs[index+i]
                          : model[index+i];

            for (int b = 0; b < 4; b++)
                payload[len + 4*i + b] = w[8*b +: 8];
        end

        len = len + 4*n;
        c = 32'hFFFFFFFF;

        send_byte(8'hA5);
        send_byte(8'h5A);

        send_byte(cmd);
        c = crc_byte(c, cmd);

        send_byte(8'(sequence_id));
        c = crc_byte(c, 8'(sequence_id));

        send_byte(8'(sequence_id >> 8));
        c = crc_byte(c, 8'(sequence_id >> 8));

        send_byte(8'(len));
        c = crc_byte(c, 8'(len));

        send_byte(8'(len >> 8));
        c = crc_byte(c, 8'(len >> 8));

        for (int i = 0; i < len; i++) begin
            send_byte(payload[i]);
            c = crc_byte(c, payload[i]);
        end

        c = c ^ 32'hFFFFFFFF;

        if (bad_crc)
            c = c ^ 1;

        for (int b = 0; b < 4; b++)
            send_byte(c[8*b +: 8]);

        base = checked;

        // Ответ имеет длину 17 байт.
        wait (received >= base + 17);

        if (
            response_bytes[base] != 8'hA5 ||
            response_bytes[base+1] != 8'h5A ||

            response_bytes[base+2] != CMD_RESPONSE ||

            {
                response_bytes[base+6],
                response_bytes[base+5]
            } != 6 ||

            response_bytes[base+8] != expected_status ||

            {
                response_bytes[base+12],
                response_bytes[base+11],
                response_bytes[base+10],
                response_bytes[base+9]
            } !== expected_value
        ) begin
            $fatal(
                1,
                "UART response mismatch cmd=%h status=%h expected=%h value=%h expected=%h",
                cmd,
                response_bytes[base+8],
                expected_status,
                {
                    response_bytes[base+12],
                    response_bytes[base+11],
                    response_bytes[base+10],
                    response_bytes[base+9]
                },
                expected_value
            );
        end

        if (bad_crc) begin
            if (
                {
                    response_bytes[base+4],
                    response_bytes[base+3]
                } != 16'hFFFF ||

                response_bytes[base+7] != 0
            )
                $fatal(1, "Bad transport-error header");

        end else if (
            {
                response_bytes[base+4],
                response_bytes[base+3]
            } != 16'(sequence_id) ||

            response_bytes[base+7] != cmd
        ) begin
            $fatal(1, "Wrong response correlation");
        end

        // Проверка CRC ответа.
        c = 32'hFFFFFFFF;

        for (int i = 2; i <= 12; i++)
            c = crc_byte(c, response_bytes[base+i]);

        if (
            {
                response_bytes[base+16],
                response_bytes[base+15],
                response_bytes[base+14],
                response_bytes[base+13]
            } !== (c ^ 32'hFFFFFFFF)
        )
            $fatal(1, "Bad response CRC");

        checked = base + 17;

        repeat (20) @(negedge clk);
    endtask

    initial begin
        $readmemh("test_model.mem", model);
        $readmemh("test_input.mem", inputs);
        $readmemh("test_expected_p6.mem", golden);

        repeat (5) @(negedge clk);
        rst = 1;

        // Начало загрузки модели.
        transaction(
            CMD_CONFIG_BEGIN,
            2652,
            0, 0, 0, 0,
            STATUS_OK,
            0
        );

        // Пакет с намеренно ошибочным CRC.
        transaction(
            CMD_CONFIG_DATA,
            0,
            64, 0, 0, 1,
            STATUS_CRC_ERROR,
            0
        );

        // Загрузка образа частями до 256 байт.
        for (int q = 0; q < 663; q += 64) begin
            transaction(
                CMD_CONFIG_DATA,
                32'(q*4),
                (663-q < 64) ? 663-q : 64,
                q,
                0,
                0,
                STATUS_OK,
                32'(((q+64 < 663) ? q+64 : 663)*4)
            );
        end

        transaction(
            CMD_CONFIG_END,
            0,
            0, 0, 0, 0,
            STATUS_OK,
            2652
        );

        if (!model_valid)
            $fatal(1, "UART-loaded model invalid");

        // Входной вектор: 49 слов FP32.
        transaction(
            CMD_INPUT_BEGIN,
            196,
            0, 0, 0, 0,
            STATUS_OK,
            0
        );

        transaction(
            CMD_INPUT_DATA,
            0,
            49, 0, 1, 0,
            STATUS_OK,
            196
        );

        transaction(
            CMD_INPUT_END,
            0,
            0, 0, 0, 0,
            STATUS_OK,
            196
        );

        // Запуск вычислений.
        transaction(
            CMD_RUN,
            0,
            0, 0, 0, 0,
            STATUS_OK,
            32'h3000
        );

        // Повторяем тот же RUN с тем же SEQ.
        // IO должен повторить ответ, а не запускать сеть заново.
        sequence_id = sequence_id - 1;

        transaction(
            CMD_RUN,
            0,
            0, 0, 0, 0,
            STATUS_OK,
            32'h3000
        );

        // Выходы первого слоя.
        for (int i = 0; i < 12; i++) begin
            transaction(
                CMD_READ_WORD,
                32'h3100 + 32'(i*4),
                0, 0, 0, 0,
                STATUS_OK,
                golden[i]
            );
        end

        // Выходы последнего слоя.
        for (int i = 0; i < 3; i++) begin
            transaction(
                CMD_READ_WORD,
                32'h3000 + 32'(i*4),
                0, 0, 0, 0,
                STATUS_OK,
                golden[12+i]
            );
        end

        // Проверка номера выбранного класса.
        transaction(
            CMD_READ_WORD,
            32'h3200,
            0, 0, 0, 0,
            STATUS_OK,
            1
        );

        $display(
            "ALL PHYSICAL UART -> SEQUENCER -> PE -> RS -> MEMORY TESTS PASSED: %0d packets",
            checked / 17
        );

        $finish;
    end

    initial begin
        #50ms;
        $fatal(1, "Physical UART test timeout");
    end

endmodule