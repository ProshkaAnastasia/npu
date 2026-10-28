`timescale 1ns/1ps

module tb_nn_processor;

    import io_types_pkg::*;

    localparam int CLK_FREQ_HZ = 100_000_000;
    localparam int UART_BAUD_RATE = 12_500_000;

    localparam time CLK_PERIOD = 10ns;
    localparam time RESPONSE_TIMEOUT = 5ms;

    localparam time UART_BIT_PERIOD =
        CLK_PERIOD * (CLK_FREQ_HZ / UART_BAUD_RATE);

    localparam MODEL_FILE = "test_model.mem";
    localparam INPUT_FILE = "test_input.mem";
    localparam EXPECTED_FILE = "test_expected_p6.mem";

    logic clk = 0;
    always #(CLK_PERIOD / 2) clk = ~clk;

    logic rst_n = 0;

    // UART-линии относительно компьютера.
    logic host_to_npu_uart = 1;
    logic npu_to_host_uart;

    logic model_valid;
    logic sequencer_busy;

    integer request_sequence = 0;
    integer received_byte_count = 0;
    integer checked_byte_count = 0;

    logic [7:0] request_payload_bytes [0:259];
    logic [7:0] received_response_bytes [0:2047];

    logic [31:0] model_image_words [0:662];
    logic [31:0] input_vector_words [0:48];
    logic [31:0] expected_output_words [0:14];

    nn_processor_top #(
        .N_PE(6),
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .UART_BAUD_RATE(UART_BAUD_RATE)
    ) dut (
        .clk_i(clk),
        .rst_n_i(rst_n),

        .uart_rx_i(host_to_npu_uart),
        .uart_tx_o(npu_to_host_uart),

        .model_valid_o(model_valid),
        .busy_o(sequencer_busy)
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

    task automatic require_file(
        input string filename
    );
        integer file_handle;

        file_handle = $fopen(filename, "r");

        if (file_handle == 0)
            $fatal(
                1,
                "Cannot open %s in simulator working directory",
                filename
            );

        $fclose(file_handle);
    endtask

    // Передача одного байта: UART 8N1.
    task automatic send_uart_byte(
        input logic [7:0] b
    );
        @(negedge clk);

        host_to_npu_uart = 0;
        #(UART_BIT_PERIOD);

        for (int i = 0; i < 8; i++) begin
            host_to_npu_uart = b[i];
            #(UART_BIT_PERIOD);
        end

        host_to_npu_uart = 1;
        #(UART_BIT_PERIOD);
    endtask

    // Независимый приёмник ответов процессора.
    initial begin : monitor
        logic [7:0] b;

        wait (rst_n);

        forever begin
            @(negedge npu_to_host_uart);
            #(UART_BIT_PERIOD / 2);

            if (npu_to_host_uart !== 0)
                $fatal(1, "Bad start bit");

            for (int i = 0; i < 8; i++) begin
                #(UART_BIT_PERIOD);
                b[i] = npu_to_host_uart;
            end

            #(UART_BIT_PERIOD);

            if (npu_to_host_uart !== 1)
                $fatal(1, "Bad stop bit");

            if ((^b) === 1'bx)
                $fatal(1, "UART response byte contains X/Z");

            if (received_byte_count >= 2048)
                $fatal(1, "UART response buffer overflow");

            received_response_bytes[received_byte_count] = b;
            received_byte_count = received_byte_count + 1;

            #(UART_BIT_PERIOD / 2);
        end
    end

    // cmd         - команда.
    // arg         - размер, смещение или адрес.
    // n           - количество передаваемых 32-битных слов.
    // index       - начальный индекс исходного массива.
    // use_input   - брать слова из входного вектора.
    // bad_crc     - намеренно испортить CRC запроса.
    // repeat_previous_request - сохранить предыдущий SEQ.
    task automatic send_request_and_check_response(
        input logic [7:0] cmd,
        input logic [31:0] arg,

        input integer n,
        input integer index,

        input logic use_input,
        input logic bad_crc,

        input logic [7:0] expected_status,
        input logic [31:0] expected_value,

        input logic repeat_previous_request = 1'b0
    );
        integer len;
        integer base;

        logic [31:0] c;
        logic [31:0] w;

        time response_deadline;

        if (!repeat_previous_request)
            request_sequence =
                (request_sequence + 1) & 16'hFFFF;

        len = 0;

        if (
            cmd == CMD_CONFIG_BEGIN ||
            cmd == CMD_INPUT_BEGIN ||
            cmd == CMD_READ_WORD ||
            cmd == CMD_CONFIG_DATA ||
            cmd == CMD_INPUT_DATA
        ) begin
            for (int b = 0; b < 4; b++)
                request_payload_bytes[b] = arg[8*b +: 8];

            len = 4;
        end

        if (n < 0 || n > 64 || index < 0)
            $fatal(
                1,
                "Invalid packet word count or source index"
            );

        if (
            n > 0 &&
            (
                (use_input && index+n > 49) ||
                (!use_input && index+n > 663)
            )
        )
            $fatal(1, "Packet source array is out of bounds");

        if (len + 4*n > 260)
            $fatal(1, "Packet payload is too large");

        for (int i = 0; i < n; i++) begin
            w = use_input
                ? input_vector_words[index+i]
                : model_image_words[index+i];

            for (int b = 0; b < 4; b++)
                request_payload_bytes[len + 4*i + b] =
                    w[8*b +: 8];
        end

        len = len + 4*n;
        c = 32'hFFFFFFFF;

        // Начало кадра в CRC не включается.
        send_uart_byte(8'hA5);
        send_uart_byte(8'h5A);

        send_uart_byte(cmd);
        c = crc_byte(c, cmd);

        send_uart_byte(8'(request_sequence));
        c = crc_byte(c, 8'(request_sequence));

        send_uart_byte(8'(request_sequence >> 8));
        c = crc_byte(c, 8'(request_sequence >> 8));

        send_uart_byte(8'(len));
        c = crc_byte(c, 8'(len));

        send_uart_byte(8'(len >> 8));
        c = crc_byte(c, 8'(len >> 8));

        for (int i = 0; i < len; i++) begin
            send_uart_byte(request_payload_bytes[i]);
            c = crc_byte(c, request_payload_bytes[i]);
        end

        c = c ^ 32'hFFFFFFFF;

        if (bad_crc)
            c = c ^ 1;

        for (int b = 0; b < 4; b++)
            send_uart_byte(c[8*b +: 8]);

        base = checked_byte_count;
        response_deadline = $time + RESPONSE_TIMEOUT;

        // Ответ занимает 17 байт.
        while (
            received_byte_count < base + 17 &&
            $time < response_deadline
        )
            @(negedge clk);

        if (received_byte_count < base + 17)
            $fatal(
                1,
                "Response timeout CMD=%h SEQ=%0d received=%0d expected=%0d",
                cmd,
                request_sequence,
                received_byte_count,
                base + 17
            );

        if (
            received_response_bytes[base] !== 8'hA5 ||
            received_response_bytes[base+1] !== 8'h5A ||

            received_response_bytes[base+2] !== CMD_RESPONSE ||

            {
                received_response_bytes[base+6],
                received_response_bytes[base+5]
            } !== 6 ||

            received_response_bytes[base+8] !== expected_status ||

            {
                received_response_bytes[base+12],
                received_response_bytes[base+11],
                received_response_bytes[base+10],
                received_response_bytes[base+9]
            } !== expected_value
        ) begin
            $fatal(
                1,
                "UART response mismatch cmd=%h status=%h expected=%h value=%h expected=%h",
                cmd,
                received_response_bytes[base+8],
                expected_status,
                {
                    received_response_bytes[base+12],
                    received_response_bytes[base+11],
                    received_response_bytes[base+10],
                    received_response_bytes[base+9]
                },
                expected_value
            );
        end

        if (bad_crc) begin
            if (
                {
                    received_response_bytes[base+4],
                    received_response_bytes[base+3]
                } !== 16'hFFFF ||

                received_response_bytes[base+7] !== 0
            )
                $fatal(1, "Bad transport-error header");

        end else if (
            {
                received_response_bytes[base+4],
                received_response_bytes[base+3]
            } !== 16'(request_sequence) ||

            received_response_bytes[base+7] !== cmd
        ) begin
            $fatal(1, "Wrong response correlation");
        end

        // Проверка CRC ответа.
        c = 32'hFFFFFFFF;

        for (int i = 2; i <= 12; i++)
            c = crc_byte(c, received_response_bytes[base+i]);

        if (
            {
                received_response_bytes[base+16],
                received_response_bytes[base+15],
                received_response_bytes[base+14],
                received_response_bytes[base+13]
            } !== (c ^ 32'hFFFFFFFF)
        )
            $fatal(1, "Bad response CRC");

        checked_byte_count = base + 17;

        repeat (20) @(negedge clk);
    endtask

    initial begin
        require_file(MODEL_FILE);
        require_file(INPUT_FILE);
        require_file(EXPECTED_FILE);
        require_file("sigmoid_lut.mem");

        $readmemh(MODEL_FILE, model_image_words);
        $readmemh(INPUT_FILE, input_vector_words);
        $readmemh(EXPECTED_FILE, expected_output_words);

        for (int i = 0; i < 663; i++)
            if ((^model_image_words[i]) === 1'bx)
                $fatal(
                    1,
                    "Model word %0d is missing or contains X/Z",
                    i
                );

        for (int i = 0; i < 49; i++)
            if ((^input_vector_words[i]) === 1'bx)
                $fatal(
                    1,
                    "Input word %0d is missing or contains X/Z",
                    i
                );

        for (int i = 0; i < 15; i++)
            if ((^expected_output_words[i]) === 1'bx)
                $fatal(
                    1,
                    "Expected word %0d is missing or contains X/Z",
                    i
                );

        if (
            model_image_words[0] !== 32'h32504E4E ||
            model_image_words[5] !== 32'd2652
        )
            $fatal(
                1,
                "This test requires the 2652-byte NNP2 example model"
            );

        repeat (5) @(negedge clk);
        rst_n = 1;

        send_request_and_check_response(
            CMD_CONFIG_BEGIN,
            2652,
            0, 0, 0, 0,
            STATUS_OK,
            0
        );

        // Намеренно неверный CRC.
        send_request_and_check_response(
            CMD_CONFIG_DATA,
            0,
            64, 0, 0, 1,
            STATUS_CRC_ERROR,
            0
        );

        // Загрузка образа частями до 256 байт.
        for (int q = 0; q < 663; q += 64) begin
            send_request_and_check_response(
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

        send_request_and_check_response(
            CMD_CONFIG_END,
            0,
            0, 0, 0, 0,
            STATUS_OK,
            2652
        );

        if (model_valid !== 1'b1)
            $fatal(1, "UART-loaded model invalid");

        // Передача 49 входных значений FP32.
        send_request_and_check_response(
            CMD_INPUT_BEGIN,
            196,
            0, 0, 0, 0,
            STATUS_OK,
            0
        );

        send_request_and_check_response(
            CMD_INPUT_DATA,
            0,
            49, 0, 1, 0,
            STATUS_OK,
            196
        );

        send_request_and_check_response(
            CMD_INPUT_END,
            0,
            0, 0, 0, 0,
            STATUS_OK,
            196
        );

        // Вычисление сети.
        send_request_and_check_response(
            CMD_RUN,
            0,
            0, 0, 0, 0,
            STATUS_OK,
            32'h3000
        );

        // Повтор того же запроса RUN с прежним SEQ.
        send_request_and_check_response(
            CMD_RUN,
            0,
            0, 0, 0, 0,
            STATUS_OK,
            32'h3000,
            1'b1
        );

        // Выходы первого слоя.
        for (int i = 0; i < 12; i++) begin
            send_request_and_check_response(
                CMD_READ_WORD,
                32'h3100 + 32'(i*4),
                0, 0, 0, 0,
                STATUS_OK,
                expected_output_words[i]
            );
        end

        // Выходы последнего слоя.
        for (int i = 0; i < 3; i++) begin
            send_request_and_check_response(
                CMD_READ_WORD,
                32'h3000 + 32'(i*4),
                0, 0, 0, 0,
                STATUS_OK,
                expected_output_words[12+i]
            );
        end

        // Номер выбранного класса.
        send_request_and_check_response(
            CMD_READ_WORD,
            32'h3200,
            0, 0, 0, 0,
            STATUS_OK,
            1
        );

        // Количество выходов.
        send_request_and_check_response(
            CMD_READ_WORD,
            32'h3204,
            0, 0, 0, 0,
            STATUS_OK,
            3
        );

        // Адрес итогового выходного буфера.
        send_request_and_check_response(
            CMD_READ_WORD,
            32'h3208,
            0, 0, 0, 0,
            STATUS_OK,
            32'h3000
        );

        // Статус завершённого вычисления.
        send_request_and_check_response(
            CMD_READ_WORD,
            32'h320C,
            0, 0, 0, 0,
            STATUS_OK,
            0
        );

        $display(
            "ALL PHYSICAL UART -> SEQUENCER -> PE -> RS -> MEMORY TESTS PASSED: %0d packets",
            checked_byte_count / 17
        );

        $finish;
    end

    initial begin
        #50ms;
        $fatal(1, "Physical UART test timeout");
    end

endmodule