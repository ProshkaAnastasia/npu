`timescale 1ns/1ps

module tb_io_controller;

    import io_types_pkg::*;

    // Ускоренный UART для симуляции. Частота clk остаётся 100 МГц.
    localparam int CLK_FREQ_HZ = 100_000_000;
    localparam int BAUD_RATE = 1_000_000;
    localparam int MAX_DATA_BYTES = 16;
    localparam int RX_TIMEOUT_CYCLES = 5_000;
    localparam time CLK_PERIOD = 10ns;
    localparam time BIT_PERIOD = CLK_PERIOD * (CLK_FREQ_HZ / BAUD_RATE);

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic uart_rx = 1'b1;
    logic uart_tx;

    io_to_seq_t io_to_seq_data;
    logic io_to_seq_valid;
    logic io_to_seq_ready = 1'b0;

    seq_to_io_t seq_to_io_data;
    logic seq_to_io_valid = 1'b0;
    logic seq_to_io_ready;

    // Буфер полезной нагрузки отправляемого пакета.
    logic [7:0] payload [0:MAX_DATA_BYTES+4];

    // Независимый монитор UART TX сохраняет принятые байты.
    logic [7:0] responses [0:1023];
    integer response_count = 0;
    integer checked_count = 0;
    integer transfer_count = 0;

    io_to_seq_t held_data;
    logic was_stalled = 1'b0;

    io_controller #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .UART_BAUD_RATE(BAUD_RATE),
        .MAX_DATA_BYTES(MAX_DATA_BYTES),
        .RX_TIMEOUT_CYCLES(RX_TIMEOUT_CYCLES)
    ) dut (
        .clk_i(clk),
        .rst_n_i(rst_n),
        .uart_rx_i(uart_rx),
        .uart_tx_o(uart_tx),
        .io_to_seq_data_o(io_to_seq_data),
        .io_to_seq_valid_o(io_to_seq_valid),
        .io_to_seq_ready_i(io_to_seq_ready),
        .seq_to_io_data_i(seq_to_io_data),
        .seq_to_io_valid_i(seq_to_io_valid),
        .seq_to_io_ready_o(seq_to_io_ready)
    );

    always #(CLK_PERIOD / 2) clk = ~clk;

    // Проверка удержания всей структуры при ready=0.
    always @(posedge clk) begin
        if (!rst_n) begin
            was_stalled <= 1'b0;
            held_data <= '0;
        end else begin
            if (was_stalled) begin
                if (io_to_seq_valid !== 1'b1 ||
                    io_to_seq_data !== held_data)
                    $fatal(1, "Data/valid changed while ready=0");
            end
            was_stalled <= io_to_seq_valid && !io_to_seq_ready;
            held_data <= io_to_seq_data;
            if (io_to_seq_valid && io_to_seq_ready)
                transfer_count = transfer_count + 1;
        end
    end

    function automatic logic [31:0] crc_byte(
        input logic [31:0] current,
        input logic [7:0] value
    );
        logic [31:0] c;
        c = current ^ {24'b0, value};
        for (int k = 0; k < 8; k++) begin
            if (c[0]) c = (c >> 1) ^ 32'hEDB88320;
            else      c = c >> 1;
        end
        return c;
    endfunction

    // Компьютер отправляет байт непосредственно на физическую RX.
    // Данные меняются между фронтами clk, чтобы избежать гонок TB/DUT.
    task automatic send_uart_byte(
        input logic [7:0] value,
        input logic bad_stop
    );
        @(negedge clk);
        uart_rx = 1'b0;
        #(BIT_PERIOD);
        for (int b = 0; b < 8; b++) begin
            uart_rx = value[b];
            #(BIT_PERIOD);
        end
        uart_rx = bad_stop ? 1'b0 : 1'b1;
        #(BIT_PERIOD);
        uart_rx = 1'b1;
    endtask

    // Приём байта с физической TX. Модуль uart_rx здесь не используется.
    task automatic receive_uart_byte(output logic [7:0] value);
        @(negedge uart_tx);
        #(BIT_PERIOD / 2);
        if (uart_tx !== 1'b0)
            $fatal(1, "Invalid TX start bit");
        for (int b = 0; b < 8; b++) begin
            #(BIT_PERIOD);
            value[b] = uart_tx;
        end
        #(BIT_PERIOD);
        if (uart_tx !== 1'b1)
            $fatal(1, "Invalid TX stop bit");
        #(BIT_PERIOD / 2);
    endtask

    initial begin : tx_monitor
        logic [7:0] value;
        wait (rst_n === 1'b1);
        forever begin
            receive_uart_byte(value);
            if ($isunknown(value))
                $fatal(1, "Unknown bit in UART response");
            if (response_count >= 1024)
                $fatal(1, "Response monitor overflow");
            responses[response_count] = value;
            response_count = response_count + 1;
        end
    end

    // PAYLOAD берётся из массива payload.
    task automatic send_packet(
        input logic [7:0] cmd,
        input logic [15:0] seq_id,
        input integer length,
        input logic corrupt_crc
    );
        logic [31:0] c;
        c = 32'hFFFFFFFF;
        send_uart_byte(8'hA5, 0);
        send_uart_byte(8'h5A, 0);
        send_uart_byte(cmd, 0);
        c = crc_byte(c, cmd);
        send_uart_byte(seq_id[7:0], 0);
        c = crc_byte(c, seq_id[7:0]);
        send_uart_byte(seq_id[15:8], 0);
        c = crc_byte(c, seq_id[15:8]);
        send_uart_byte(8'(length), 0);
        c = crc_byte(c, 8'(length));
        send_uart_byte(8'(length >> 8), 0);
        c = crc_byte(c, 8'(length >> 8));
        for (int i = 0; i < length; i++) begin
            send_uart_byte(payload[i], 0);
            c = crc_byte(c, payload[i]);
        end
        c = c ^ 32'hFFFFFFFF;
        if (corrupt_crc) c = c ^ 32'd1;
        for (int i = 0; i < 4; i++)
            send_uart_byte(c[8*i +: 8], 0);
    endtask

    task automatic set_payload_u32(
        input integer base,
        input logic [31:0] value
    );
        for (int i = 0; i < 4; i++)
            payload[base+i] = value[8*i +: 8];
    endtask

    // Проверка описания команды до handshake.
    task automatic accept_header(
        input logic [7:0] cmd,
        input logic [15:0] seq_id,
        input logic [31:0] image_size,
        input logic [31:0] offset,
        input logic [15:0] data_length
    );
        wait (io_to_seq_valid === 1'b1);
        @(negedge clk);
        if (io_to_seq_data.is_header !== 1'b1 ||
            io_to_seq_data.cmd !== cmd ||
            io_to_seq_data.seq !== seq_id ||
            io_to_seq_data.image_size !== image_size ||
            io_to_seq_data.offset !== offset ||
            io_to_seq_data.data_length !== data_length ||
            io_to_seq_data.keep !== 4'b0000 ||
            io_to_seq_data.last !== 1'b0)
            $fatal(1, "Invalid command header, seq=%0d", seq_id);
        repeat (12) @(negedge clk);
        io_to_seq_ready = 1'b1;
        @(negedge clk);
        io_to_seq_ready = 1'b0;
    endtask

    task automatic accept_word(
        input logic [31:0] word_value,
        input logic [3:0] keep,
        input logic last
    );
        wait (io_to_seq_valid === 1'b1);
        @(negedge clk);
        if (io_to_seq_data.is_header !== 1'b0 ||
            io_to_seq_data.word !== word_value ||
            io_to_seq_data.keep !== keep ||
            io_to_seq_data.last !== last)
            $fatal(1, "Invalid word: received=%h expected=%h",
                   io_to_seq_data.word, word_value);
        repeat (8) @(negedge clk);
        io_to_seq_ready = 1'b1;
        @(negedge clk);
        io_to_seq_ready = 1'b0;
    endtask

    // Имитируем завершение записи/проверки секвенсором.
    task automatic sequencer_reply(
        input logic [15:0] seq_id,
        input logic [7:0] status,
        input logic [31:0] offset
    );
        @(negedge clk);
        seq_to_io_data.seq = seq_id;
        seq_to_io_data.status = status;
        seq_to_io_data.next_offset = offset;
        seq_to_io_valid = 1'b1;
        do @(posedge clk); while (seq_to_io_ready !== 1'b1);
        @(negedge clk);
        seq_to_io_valid = 1'b0;
    endtask

    // Проверка всего ответного пакета, включая его CRC.
    task automatic expect_response(
        input logic [15:0] seq_id,
        input logic [7:0] cmd,
        input logic [7:0] status,
        input logic [31:0] offset
    );
        logic [31:0] c;
        logic [31:0] received;
        integer base;
        base = checked_count;
        wait (response_count >= base + 17);
        if (response_count != base + 17)
            $fatal(1, "Unexpected extra UART response bytes");
        if (responses[base] !== 8'hA5 ||
            responses[base+1] !== 8'h5A ||
            responses[base+2] !== CMD_RESPONSE ||
            {responses[base+4], responses[base+3]} !== seq_id ||
            {responses[base+6], responses[base+5]} !== 16'd6 ||
            responses[base+7] !== cmd ||
            responses[base+8] !== status ||
            {responses[base+12], responses[base+11],
             responses[base+10], responses[base+9]} !== offset)
            $fatal(1, "Invalid response fields, expected seq=%0d status=%0d",
                   seq_id, status);
        c = 32'hFFFFFFFF;
        for (int i = 2; i <= 12; i++)
            c = crc_byte(c, responses[base+i]);
        received = {responses[base+16], responses[base+15],
                    responses[base+14], responses[base+13]};
        if (received !== (c ^ 32'hFFFFFFFF))
            $fatal(1, "Response CRC mismatch");
        checked_count = base + 17;
        repeat (20) @(negedge clk);
    endtask

    task automatic expect_no_forwarding(input integer before_count);
        if (transfer_count != before_count || io_to_seq_valid !== 1'b0)
            $fatal(1, "Rejected/duplicate packet reached sequencer");
    endtask

    initial begin : tests
        integer before_count;
        logic [31:0] crc_check;

        seq_to_io_data = '0;
        for (int i = 0; i <= MAX_DATA_BYTES+4; i++) payload[i] = 0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);
        if (uart_tx !== 1'b1 || io_to_seq_valid !== 1'b0)
            $fatal(1, "Invalid state after reset");

        // Стандартный контрольный вектор CRC: ASCII "123456789".
        crc_check = 32'hFFFFFFFF;
        for (int i = 0; i < 9; i++)
            crc_check = crc_byte(crc_check, 8'h31 + 8'(i));
        if ((crc_check ^ 32'hFFFFFFFF) !== 32'hCBF43926)
            $fatal(1, "Testbench CRC implementation is incorrect");

        // 1. BEGIN: ACK запрещён до результата секвенсора.
        set_payload_u32(0, 21);
        send_packet(CMD_CONFIG_BEGIN, 1, 4, 0);
        accept_header(CMD_CONFIG_BEGIN, 1, 21, 0, 0);
        repeat (200) @(negedge clk);
        if (response_count != 0 || uart_tx !== 1'b1)
            $fatal(1, "ACK sent before sequencer completion");
        sequencer_reply(1, STATUS_OK, 0);
        expect_response(1, CMD_CONFIG_BEGIN, STATUS_OK, 0);
        $display("PASS 1: BEGIN, header stall, ACK after completion");

        // 2. DATA: пять байтов, два слова, ожидание ready.
        set_payload_u32(0, 0);
        payload[4]=8'h11; payload[5]=8'h22; payload[6]=8'h33;
        payload[7]=8'h44; payload[8]=8'h55;
        send_packet(CMD_CONFIG_DATA, 2, 9, 0);
        accept_header(CMD_CONFIG_DATA, 2, 0, 0, 5);
        // Нарушение протокола хостом: лишний байт при занятом буфере.
        // Содержимое ожидающего слова должно сохраниться.
        send_uart_byte(8'hEF, 0);
        repeat (20) @(negedge clk);
        accept_word(32'h44332211, 4'b1111, 0);
        accept_word(32'h00000055, 4'b0001, 1);
        repeat (200) @(negedge clk);
        if (response_count != checked_count || uart_tx !== 1'b1)
            $fatal(1, "DATA acknowledged before write completion");
        sequencer_reply(2, STATUS_OK, 5);
        expect_response(2, CMD_CONFIG_DATA, STATUS_OK, 5);
        $display("PASS 2: DATA, partial word, backpressure, buffer protection");

        // 3. Повтор уже выполненного DATA не исполняется снова.
        before_count = transfer_count;
        send_packet(CMD_CONFIG_DATA, 2, 9, 0);
        expect_response(2, CMD_CONFIG_DATA, STATUS_OK, 5);
        expect_no_forwarding(before_count);
        $display("PASS 3: duplicate request replays response");

        // 4. Тот же SEQ с другим содержимым.
        payload[4] = 8'h99;
        send_packet(CMD_CONFIG_DATA, 2, 9, 0);
        expect_response(2, CMD_CONFIG_DATA, STATUS_SEQ_ERROR, 0);
        expect_no_forwarding(before_count);
        $display("PASS 4: conflicting SEQ rejected");

        // 5. Повреждённая CRC не допускает данные к секвенсору.
        send_packet(CMD_CONFIG_DATA, 3, 9, 1);
        expect_response(16'hFFFF, 0, STATUS_CRC_ERROR, 0);
        expect_no_forwarding(before_count);
        // Повторяем именно этот запрос с исправленной CRC.
        send_packet(CMD_CONFIG_DATA, 3, 9, 0);
        accept_header(CMD_CONFIG_DATA, 3, 0, 0, 5);
        accept_word(32'h44332299, 4'b1111, 0);
        accept_word(32'h00000055, 4'b0001, 1);
        sequencer_reply(3, STATUS_OK, 5);
        expect_response(3, CMD_CONFIG_DATA, STATUS_OK, 5);
        $display("PASS 5: CRC rejection and successful retry");

        // 6. Полный буфер: 16 байтов, четыре полных слова.
        set_payload_u32(0, 5);
        for (int i = 0; i < MAX_DATA_BYTES; i++) payload[4+i] = 8'(i);
        send_packet(CMD_CONFIG_DATA, 4, MAX_DATA_BYTES+4, 0);
        accept_header(CMD_CONFIG_DATA, 4, 0, 5, MAX_DATA_BYTES);
        accept_word(32'h03020100, 4'b1111, 0);
        accept_word(32'h07060504, 4'b1111, 0);
        accept_word(32'h0B0A0908, 4'b1111, 0);
        accept_word(32'h0F0E0D0C, 4'b1111, 1);
        sequencer_reply(4, STATUS_OK, 21);
        expect_response(4, CMD_CONFIG_DATA, STATUS_OK, 21);
        $display("PASS 6: maximum payload");

        // 7. Некорректные длины и неизвестная команда.
        before_count = transfer_count;
        send_packet(CMD_CONFIG_DATA, 5, MAX_DATA_BYTES+5, 0);
        expect_response(5, CMD_CONFIG_DATA, STATUS_FORMAT_ERROR, 0);
        send_packet(8'h77, 6, 0, 0);
        expect_response(6, 8'h77, STATUS_FORMAT_ERROR, 0);
        send_packet(CMD_CONFIG_END, 7, 1, 0);
        expect_response(7, CMD_CONFIG_END, STATUS_FORMAT_ERROR, 0);
        send_packet(CMD_CONFIG_DATA, 8, 4, 0);
        expect_response(8, CMD_CONFIG_DATA, STATUS_FORMAT_ERROR, 0);
        expect_no_forwarding(before_count);
        $display("PASS 7: invalid lengths and unknown command");

        // 8. Заголовок начался, но передача оборвалась.
        send_uart_byte(8'hA5, 0);
        send_uart_byte(8'h5A, 0);
        expect_response(16'hFFFF, 0, STATUS_TIMEOUT, 0);
        expect_no_forwarding(before_count);
        $display("PASS 8: inter-byte timeout");

        // 9. Ошибка стопового бита во время заголовка.
        send_uart_byte(8'hA5, 0);
        send_uart_byte(8'h5A, 0);
        send_uart_byte(CMD_CONFIG_BEGIN, 1);
        expect_response(16'hFFFF, 0, STATUS_FORMAT_ERROR, 0);
        expect_no_forwarding(before_count);
        $display("PASS 9: UART framing error");

        // 10. DATA минимальной длины и отказ секвенсора.
        set_payload_u32(0, 21);
        payload[4] = 8'hA5;
        send_packet(CMD_CONFIG_DATA, 9, 5, 0);
        accept_header(CMD_CONFIG_DATA, 9, 0, 21, 1);
        // Секвенсор принимает, но не записывает отклонённые данные.
        accept_word(32'h000000A5, 4'b0001, 1);
        sequencer_reply(9, 8'd16, 21);
        expect_response(9, CMD_CONFIG_DATA, 8'd16, 21);
        $display("PASS 10: minimum DATA, sequencer error propagation");

        // 11. END и повтор END после потери ответа.
        send_packet(CMD_CONFIG_END, 10, 0, 0);
        accept_header(CMD_CONFIG_END, 10, 0, 0, 0);
        sequencer_reply(10, STATUS_OK, 21);
        expect_response(10, CMD_CONFIG_END, STATUS_OK, 21);
        before_count = transfer_count;
        send_packet(CMD_CONFIG_END, 10, 0, 0);
        expect_response(10, CMD_CONFIG_END, STATUS_OK, 21);
        expect_no_forwarding(before_count);
        $display("PASS 11: END and duplicate END");

        repeat (RX_TIMEOUT_CYCLES + 100) @(negedge clk);
        if (response_count != checked_count || io_to_seq_valid !== 1'b0)
            $fatal(1, "Unexpected activity after tests");
        $display("ALL TESTS PASSED: %0d checked responses, %0d sequencer transfers",
                 checked_count / 17, transfer_count);
        $finish;
    end

    // Защита от зависания любого wait в тесте.
    initial begin
        #20ms;
        $fatal(1, "Global testbench timeout");
    end

endmodule
