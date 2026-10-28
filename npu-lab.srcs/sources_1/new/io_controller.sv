// Формат UART-пакета:
//
// A5 5A | CMD | SEQ[LE16] | LEN[LE16] | PAYLOAD | CRC32[LE32]
//
// LEN - количество байтов PAYLOAD.
// Многобайтовые числа передаются little-endian.
//
// CRC-32/ISO-HDLC:
//   reflected polynomial = 0xEDB88320
//   initial              = 0xFFFFFFFF
//   final XOR            = 0xFFFFFFFF
//
// CRC покрывает CMD, SEQ, LEN и PAYLOAD.
//
// Команды:
// CONFIG_BEGIN:
//   PAYLOAD = image_size[LE32]
//
// CONFIG_DATA:
//   PAYLOAD = offset[LE32] + байты образа
//
// CONFIG_END:
//   PAYLOAD отсутствует
//
// Ответ:
// CMD = CMD_RESPONSE
// PAYLOAD = request_cmd[8] + status[8] + next_offset[LE32]
//
// Компьютер отправляет один запрос и ожидает ответ.
// При повторе запроса сохраняет SEQ и содержимое.
// Для нового запроса использует новый SEQ.
//
// Секвенсор:
// - принимает описание команды;
// - для CONFIG_DATA принимает все слова данных;
// - возвращает результат после завершения обработки.
//
// Если CONFIG_DATA отклонена, секвенсор всё равно принимает
// и отбрасывает слова, затем возвращает ошибку.
//
// Проверку CRC полного образа, адресов и model_valid
// выполняет секвенсор.
`timescale 1ns/1ps

module io_controller #(
    parameter int CLK_FREQ_HZ = 100_000_000,
    parameter int UART_BAUD_RATE = 115_200,

    // Максимальное количество байт собственно данных
    // в CONFIG_DATA / INPUT_DATA, без поля offset.
    parameter int MAX_DATA_BYTES = 256,

    // Максимальный интервал между принятыми байтами пакета.
    // При 100 MHz значение 1_000_000 соответствует 10 ms.
    // Ноль отключает тайм-аут.
    parameter int RX_TIMEOUT_CYCLES = 1_000_000
) (
    input logic clk_i,
    input logic rst_n_i,

    input  logic uart_rx_i,
    output logic uart_tx_o,

    output io_types_pkg::io_to_seq_t io_to_seq_data_o,
    output logic io_to_seq_valid_o,
    input  logic io_to_seq_ready_i,

    input  io_types_pkg::seq_to_io_t seq_to_io_data_i,
    input  logic seq_to_io_valid_i,
    output logic seq_to_io_ready_o
);

    import io_types_pkg::*;

    logic [7:0] rx_byte;
    logic rx_valid;
    logic rx_error;

    logic [7:0] tx_byte;
    logic tx_valid;
    logic tx_ready;

    uart_rx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE(UART_BAUD_RATE)
    ) u_uart_rx (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),

        .rx_i(uart_rx_i),

        .byte_o(rx_byte),
        .valid_o(rx_valid),
        .error_o(rx_error)
    );

    uart_tx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE(UART_BAUD_RATE)
    ) u_uart_tx (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),

        .byte_i(tx_byte),
        .valid_i(tx_valid),
        .ready_o(tx_ready),

        .tx_o(uart_tx_o)
    );

    // Четыре дополнительных байта - поле offset.
    localparam int MAX_PAYLOAD = MAX_DATA_BYTES + 4;

    typedef enum logic [3:0] {
        SYNC0,
        SYNC1,
        HEADER,
        PAYLOAD,
        RXCRC,
        CHECK,

        COMMAND,
        STREAM,
        WAIT_RESULT,

        SEND
    } state_t;

    state_t state;

    logic [7:0] payload [0:MAX_PAYLOAD-1];

    logic [7:0] command;
    logic [15:0] sequence_id;
    logic [15:0] length;

    logic [31:0] rx_crc;
    logic [31:0] received_crc;
    logic [31:0] packet_crc;

    integer header_pos;
    integer payload_pos;
    integer crc_pos;
    integer stream_pos;
    integer tx_pos;
    integer timeout_count;

    logic [31:0] tx_crc;

    logic [15:0] response_seq;
    logic [7:0] response_cmd;
    logic [7:0] response_status;
    logic [31:0] response_offset;

    // Сохранённый ответ на последний обработанный запрос.
    // Повтор идентичного запроса с тем же SEQ
    // не должен выполнять команду повторно.
    logic cache_valid;

    logic [15:0] cache_seq;
    logic [15:0] cache_length;

    logic [7:0] cache_cmd;
    logic [7:0] cache_status;

    logic [31:0] cache_crc;
    logic [31:0] cache_offset;

    logic receiving;

    logic [31:0] stream_word;
    logic [3:0] stream_keep;

    function automatic logic [31:0] crc_byte(
        input logic [31:0] current,
        input logic [7:0] value
    );
        logic [31:0] c;

        c = current ^ {24'b0, value};

        for (int k = 0; k < 8; k++)
            c = c[0] ? ((c >> 1) ^ 32'hEDB88320)
                     :  (c >> 1);

        return c;
    endfunction

    // Чтение uint32 из payload, little-endian.
    function automatic logic [31:0] payload_u32(
        input integer base
    );
        return {
            payload[base+3],
            payload[base+2],
            payload[base+1],
            payload[base]
        };
    endfunction

    // Ответ:
    // A5 5A CMD_RESPONSE SEQ[15:0] LEN[15:0]
    // request_cmd status value[31:0] CRC[31:0].
    //
    // Всего 17 байт; LEN = 6.
    task automatic start_response(
        input logic [15:0] seq_id,
        input logic [7:0] cmd,
        input logic [7:0] status,
        input logic [31:0] offset
    );
        response_seq <= seq_id;
        response_cmd <= cmd;
        response_status <= status;
        response_offset <= offset;

        tx_crc <= 32'hFFFFFFFF;
        tx_pos <= 0;

        state <= SEND;
    endtask

    always_comb begin
        receiving =
            state == SYNC1 ||
            state == HEADER ||
            state == PAYLOAD ||
            state == RXCRC;

        io_to_seq_data_o = '0;

        io_to_seq_data_o.is_header = (state == COMMAND);
        io_to_seq_data_o.cmd = command;
        io_to_seq_data_o.seq = sequence_id;

        // BEGIN: размер образа или входного вектора.
        if (
            command == CMD_CONFIG_BEGIN ||
            command == CMD_INPUT_BEGIN
        )
            io_to_seq_data_o.image_size = payload_u32(0);

        // DATA: смещение и длина собственно данных.
        if (
            command == CMD_CONFIG_DATA ||
            command == CMD_INPUT_DATA
        ) begin
            io_to_seq_data_o.offset = payload_u32(0);
            io_to_seq_data_o.data_length = length - 16'd4;
        end

        // READ_WORD: адрес общей памяти.
        if (command == CMD_READ_WORD)
            io_to_seq_data_o.offset = payload_u32(0);

        stream_word = '0;
        stream_keep = '0;

        for (int b = 0; b < 4; b++) begin
            if (
                state == STREAM &&
                stream_pos + b < int'(length)
            ) begin
                stream_word[8*b +: 8] = payload[stream_pos+b];
                stream_keep[b] = 1'b1;
            end
        end

        io_to_seq_data_o.word = stream_word;
        io_to_seq_data_o.keep = stream_keep;

        io_to_seq_data_o.last =
            state == STREAM &&
            stream_pos + 4 >= int'(length);

        io_to_seq_valid_o =
            state == COMMAND ||
            state == STREAM;

        seq_to_io_ready_o = (state == WAIT_RESULT);

        tx_valid = (state == SEND);
        tx_byte = 8'h00;

        case (tx_pos)
            0: tx_byte = 8'hA5;
            1: tx_byte = 8'h5A;

            2: tx_byte = CMD_RESPONSE;

            3: tx_byte = response_seq[7:0];
            4: tx_byte = response_seq[15:8];

            5: tx_byte = 8'd6;
            6: tx_byte = 8'd0;

            7: tx_byte = response_cmd;
            8: tx_byte = response_status;

            9:  tx_byte = response_offset[7:0];
            10: tx_byte = response_offset[15:8];
            11: tx_byte = response_offset[23:16];
            12: tx_byte = response_offset[31:24];

            13: tx_byte = tx_crc[7:0]   ^ 8'hFF;
            14: tx_byte = tx_crc[15:8]  ^ 8'hFF;
            15: tx_byte = tx_crc[23:16] ^ 8'hFF;
            16: tx_byte = tx_crc[31:24] ^ 8'hFF;

            default: tx_byte = 8'h00;
        endcase
    end

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            state <= SYNC0;

            command <= '0;
            sequence_id <= '0;
            length <= '0;

            rx_crc <= 32'hFFFFFFFF;
            received_crc <= '0;
            packet_crc <= '0;

            header_pos <= 0;
            payload_pos <= 0;
            crc_pos <= 0;
            stream_pos <= 0;
            tx_pos <= 0;
            timeout_count <= 0;

            tx_crc <= 32'hFFFFFFFF;

            response_seq <= '0;
            response_cmd <= '0;
            response_status <= '0;
            response_offset <= '0;

            cache_valid <= 1'b0;
            cache_seq <= '0;
            cache_length <= '0;
            cache_cmd <= '0;
            cache_status <= '0;
            cache_crc <= '0;
            cache_offset <= '0;

        end else begin
            if (!receiving || rx_valid)
                timeout_count <= 0;
            else if (RX_TIMEOUT_CYCLES > 0)
                timeout_count <= timeout_count + 1;

            if (receiving && rx_error) begin
                timeout_count <= 0;

                start_response(
                    16'hFFFF,
                    8'd0,
                    STATUS_FORMAT_ERROR,
                    32'd0
                );

            end else if (
                receiving &&
                !rx_valid &&
                RX_TIMEOUT_CYCLES > 0 &&
                timeout_count >= RX_TIMEOUT_CYCLES-1
            ) begin
                timeout_count <= 0;

                start_response(
                    16'hFFFF,
                    8'd0,
                    STATUS_TIMEOUT,
                    32'd0
                );

            end else begin
                case (state)

                    SYNC0: begin
                        if (
                            rx_valid &&
                            !rx_error &&
                            rx_byte == 8'hA5
                        )
                            state <= SYNC1;
                    end

                    SYNC1: begin
                        if (rx_valid) begin
                            if (rx_byte == 8'h5A) begin
                                state <= HEADER;
                                header_pos <= 0;
                                rx_crc <= 32'hFFFFFFFF;

                            end else if (rx_byte != 8'hA5) begin
                                state <= SYNC0;
                            end
                        end
                    end

                    HEADER: begin
                        if (rx_valid) begin
                            rx_crc <= crc_byte(rx_crc, rx_byte);

                            case (header_pos)
                                0:
                                    command <= rx_byte;

                                1:
                                    sequence_id[7:0] <= rx_byte;

                                2:
                                    sequence_id[15:8] <= rx_byte;

                                3:
                                    length[7:0] <= rx_byte;

                                4: begin
                                    length[15:8] <= rx_byte;

                                    payload_pos <= 0;
                                    crc_pos <= 0;
                                    received_crc <= '0;

                                    if ({rx_byte, length[7:0]} == 0)
                                        state <= RXCRC;
                                    else
                                        state <= PAYLOAD;
                                end

                                default: ;
                            endcase

                            header_pos <= header_pos + 1;
                        end
                    end

                    PAYLOAD: begin
                        if (rx_valid) begin
                            // Слишком большой пакет дочитывается,
                            // но за пределы буфера запись не выполняется.
                            if (payload_pos < MAX_PAYLOAD)
                                payload[payload_pos] <= rx_byte;

                            rx_crc <= crc_byte(rx_crc, rx_byte);

                            if (payload_pos + 1 == int'(length))
                                state <= RXCRC;

                            payload_pos <= payload_pos + 1;
                        end
                    end

                    RXCRC: begin
                        if (rx_valid) begin
                            received_crc[8*crc_pos +: 8] <= rx_byte;

                            if (crc_pos == 3)
                                state <= CHECK;
                            else
                                crc_pos <= crc_pos + 1;
                        end
                    end

                    CHECK: begin
                        packet_crc <= received_crc;

                        if (
                            received_crc !=
                            (rx_crc ^ 32'hFFFFFFFF)
                        ) begin
                            start_response(
                                16'hFFFF,
                                8'd0,
                                STATUS_CRC_ERROR,
                                32'd0
                            );

                        end else if (
                            int'(length) > MAX_PAYLOAD ||
                            !(
                                (
                                    (
                                        command == CMD_CONFIG_BEGIN ||
                                        command == CMD_INPUT_BEGIN ||
                                        command == CMD_READ_WORD
                                    ) &&
                                    length == 4
                                ) ||
                                (
                                    (
                                        command == CMD_CONFIG_DATA ||
                                        command == CMD_INPUT_DATA
                                    ) &&
                                    length > 4
                                ) ||
                                (
                                    (
                                        command == CMD_CONFIG_END ||
                                        command == CMD_INPUT_END ||
                                        command == CMD_RUN
                                    ) &&
                                    length == 0
                                )
                            )
                        ) begin
                            start_response(
                                sequence_id,
                                command,
                                STATUS_FORMAT_ERROR,
                                32'd0
                            );

                        end else if (
                            cache_valid &&
                            sequence_id == cache_seq
                        ) begin
                            if (
                                command == cache_cmd &&
                                length == cache_length &&
                                received_crc == cache_crc
                            ) begin
                                start_response(
                                    cache_seq,
                                    cache_cmd,
                                    cache_status,
                                    cache_offset
                                );
                            end else begin
                                start_response(
                                    sequence_id,
                                    command,
                                    STATUS_SEQ_ERROR,
                                    32'd0
                                );
                            end

                        end else begin
                            state <= COMMAND;
                        end
                    end

                    COMMAND: begin
                        if (
                            io_to_seq_valid_o &&
                            io_to_seq_ready_i
                        ) begin
                            // Первые четыре байта - offset.
                            stream_pos <= 4;

                            if (
                                command == CMD_CONFIG_DATA ||
                                command == CMD_INPUT_DATA
                            )
                                state <= STREAM;
                            else
                                state <= WAIT_RESULT;
                        end
                    end

                    STREAM: begin
                        if (
                            io_to_seq_valid_o &&
                            io_to_seq_ready_i
                        ) begin
                            if (io_to_seq_data_o.last)
                                state <= WAIT_RESULT;

                            stream_pos <= stream_pos + 4;
                        end
                    end

                    WAIT_RESULT: begin
                        if (
                            seq_to_io_valid_i &&
                            seq_to_io_ready_o
                        ) begin
                            cache_valid <= 1'b1;

                            cache_seq <= sequence_id;
                            cache_cmd <= command;
                            cache_length <= length;
                            cache_crc <= packet_crc;

                            cache_status <= seq_to_io_data_i.status;
                            cache_offset <=
                                seq_to_io_data_i.next_offset;

                            start_response(
                                sequence_id,
                                command,
                                seq_to_io_data_i.status,
                                seq_to_io_data_i.next_offset
                            );
                        end
                    end

                    SEND: begin
                        if (tx_valid && tx_ready) begin
                            // CRC ответа: CMD, SEQ, LEN, PAYLOAD.
                            if (tx_pos >= 2 && tx_pos <= 12)
                                tx_crc <= crc_byte(tx_crc, tx_byte);

                            if (tx_pos == 16)
                                state <= SYNC0;
                            else
                                tx_pos <= tx_pos + 1;
                        end
                    end

                    default:
                        state <= SYNC0;

                endcase
            end
        end
    end

endmodule