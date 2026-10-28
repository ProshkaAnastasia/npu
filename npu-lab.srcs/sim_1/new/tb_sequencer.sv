`timescale 1ns/1ps

module tb_sequencer #(
    parameter int P = 6
);

    import io_types_pkg::*;
    import memory_types_pkg::*;
    import pe_types_pkg::*;
    import rs_types_pkg::*;

    logic clk = 0;
    always #5 clk = ~clk;

    logic rst_n = 0;

    // Эмуляция интерфейса IO -> секвенсор.
    io_to_seq_t io_to_seq_request;
    logic io_to_seq_request_valid = 0;
    logic seq_to_io_request_ready;

    // Ответ секвенсора эмулятору IO.
    seq_to_io_t seq_to_io_response;
    logic seq_to_io_response_valid;
    logic io_to_seq_response_ready = 0;

    seq_to_io_t held_io_response;
    logic io_response_stalled_reg = 0;

    // Интерфейс shared memory со стороны секвенсора.
    wire mem_req_t seq_to_shared_mem_request;
    wire seq_to_shared_mem_request_valid;
    wire shared_mem_to_seq_request_ready;

    wire mem_rsp_t shared_mem_to_seq_response;
    wire shared_mem_to_seq_response_valid;
    wire seq_to_shared_mem_response_ready;

    // Интерфейс памяти до искусственных задержек.
    wire mem_rsp_t memory_response_ungated;
    wire memory_request_ready_ungated;
    wire memory_response_valid_ungated;
    wire memory_response_ready_gated;

    // Внесение ошибки ответа на запись по адресу 0.
    logic inject_write_error = 0;

    logic last_memory_request_was_write = 0;
    logic [31:0] last_memory_request_address = 0;

    // Секвенсор -> PE.
    wire seq_to_pe_t seq_to_pe_command [0:P-1];

    wire [P-1:0] seq_to_pe_command_valid;
    wire [P-1:0] pe_to_seq_command_ready;
    wire [P-1:0] pe_command_ready_ungated;

    // PE -> секвенсор.
    wire [P-1:0] pe_to_seq_layer_done;
    wire [P-1:0] pe_to_seq_sum_valid;
    wire [P-1:0] pe_sum_valid_for_seq;

    wire pe_status_t pe_to_seq_status [0:P-1];

    // PE -> reduce.
    wire [31:0] pe_to_reduce_sum [0:P-1];

    // Секвенсор -> RS.
    wire [31:0] seq_to_rs_bias;
    wire seq_to_rs_start;
    wire rs_to_seq_start_ready;
    wire rs_start_ready_ungated;

    // RS -> секвенсор.
    wire [31:0] rs_to_seq_result_data;
    wire rs_status_t rs_to_seq_result_status;

    wire rs_to_seq_result_valid;
    wire seq_to_rs_result_ready;
    wire rs_result_valid_ungated;

    wire compute_rst_n;
    wire model_valid;
    wire sequencer_busy;

    integer cycle_count = 0;
    integer request_sequence = 0;
    integer checked_response_count = 0;

    logic [31:0] original_model_words [0:662];
    logic [31:0] model_image_words [0:662];
    logic [31:0] input_vector_words [0:48];
    logic [31:0] expected_output_words [0:14];

    logic [31:0] model_crc_work;

    logic inference_in_progress = 0;
    logic result_write_pending = 0;

    integer accepted_neuron_result_count = 0;

    // Искусственные задержки интерфейсов.
    wire memory_request_path_enabled =
        (cycle_count % 5 != 1);

    wire memory_response_path_enabled =
        (cycle_count % 7 != 2);

    wire rs_start_path_enabled =
        (cycle_count % 4 != 1);

    wire rs_result_path_enabled =
        (cycle_count % 6 != 3);

    // Дополнительно задерживаем видимость готовности сумм.
    // Секвенсор обязан ждать, даже если layer_done уже установлен.
    wire sum_visibility_enabled =
        (cycle_count % 7 != 5);

    assign pe_sum_valid_for_seq =
        pe_to_seq_sum_valid & {P{sum_visibility_enabled}};

    assign shared_mem_to_seq_response.value =
        memory_response_ungated.value;

    assign shared_mem_to_seq_response.error =
        memory_response_ungated.error ||
        (
            inject_write_error &&
            last_memory_request_was_write &&
            last_memory_request_address == 0
        );

    sequencer #(
        .N_PE(P)
    ) dut (
        .clk_i(clk),
        .rst_n_i(rst_n),

        .io_to_seq_data_i(io_to_seq_request),
        .io_to_seq_valid_i(io_to_seq_request_valid),
        .io_to_seq_ready_o(seq_to_io_request_ready),

        .seq_to_io_data_o(seq_to_io_response),
        .seq_to_io_valid_o(seq_to_io_response_valid),
        .seq_to_io_ready_i(io_to_seq_response_ready),

        .seq_to_mem_data_o(seq_to_shared_mem_request),
        .seq_to_mem_valid_o(seq_to_shared_mem_request_valid),
        .seq_to_mem_ready_i(shared_mem_to_seq_request_ready),

        .mem_to_seq_data_i(shared_mem_to_seq_response),
        .mem_to_seq_valid_i(shared_mem_to_seq_response_valid),
        .mem_to_seq_ready_o(seq_to_shared_mem_response_ready),

        .seq_to_pe_command_o(seq_to_pe_command),
        .seq_to_pe_command_valid_o(seq_to_pe_command_valid),
        .pe_to_seq_command_ready_i(pe_to_seq_command_ready),

        .pe_to_seq_layer_done_i(pe_to_seq_layer_done),
        .pe_to_seq_sum_valid_i(pe_sum_valid_for_seq),
        .pe_to_seq_status_i(pe_to_seq_status),

        .seq_to_rs_bias_o(seq_to_rs_bias),
        .seq_to_rs_start_o(seq_to_rs_start),
        .rs_to_seq_start_ready_i(rs_to_seq_start_ready),

        .rs_to_seq_result_data_i(rs_to_seq_result_data),
        .rs_to_seq_result_status_i(rs_to_seq_result_status),
        .rs_to_seq_result_valid_i(rs_to_seq_result_valid),
        .seq_to_rs_result_ready_o(seq_to_rs_result_ready),

        .compute_rst_n_o(compute_rst_n),
        .model_valid_o(model_valid),
        .busy_o(sequencer_busy)
    );

    // Обе стороны handshake должны видеть одинаковую задержку.
    assign shared_mem_to_seq_request_ready =
        memory_request_ready_ungated &&
        memory_request_path_enabled;

    assign shared_mem_to_seq_response_valid =
        memory_response_valid_ungated &&
        memory_response_path_enabled;

    assign memory_response_ready_gated =
        seq_to_shared_mem_response_ready &&
        memory_response_path_enabled;

    memory #(
        .MEM_WORDS(4096)
    ) shared_memory (
        .clk_i(clk),
        .rst_n_i(rst_n),

        .a_client_to_mem_request_data_i(
            seq_to_shared_mem_request
        ),
        .a_client_to_mem_request_valid_i(
            seq_to_shared_mem_request_valid &&
            memory_request_path_enabled
        ),
        .a_mem_to_client_request_ready_o(
            memory_request_ready_ungated
        ),

        .a_mem_to_client_response_data_o(
            memory_response_ungated
        ),
        .a_mem_to_client_response_valid_o(
            memory_response_valid_ungated
        ),
        .a_client_to_mem_response_ready_i(
            memory_response_ready_gated
        ),

        .b_client_to_mem_request_data_i('0),
        .b_client_to_mem_request_valid_i(1'b0),
        .b_mem_to_client_request_ready_o(),

        .b_mem_to_client_response_data_o(),
        .b_mem_to_client_response_valid_o(),
        .b_client_to_mem_response_ready_i(1'b1)
    );

    for (genvar p = 0; p < P; p++) begin : g_pe

        wire pe_command_path_enabled =
            ((cycle_count + p) % 4 != 0);

        assign pe_to_seq_command_ready[p] =
            pe_command_ready_ungated[p] &&
            pe_command_path_enabled;

        pe u_pe (
            .clk_i(clk),
            .rst_n_i(compute_rst_n),

            .seq_to_pe_command_i(seq_to_pe_command[p]),
            .seq_to_pe_command_valid_i(
                seq_to_pe_command_valid[p] &&
                pe_command_path_enabled
            ),
            .pe_to_seq_command_ready_o(
                pe_command_ready_ungated[p]
            ),

            .pe_to_reduce_sum_o(pe_to_reduce_sum[p]),
            .pe_to_seq_sum_valid_o(pe_to_seq_sum_valid[p]),

            .pe_to_seq_layer_done_o(pe_to_seq_layer_done[p]),
            .pe_to_seq_status_o(pe_to_seq_status[p])
        );

        wire seq_to_pe_t observed_pe_command;
        assign observed_pe_command = seq_to_pe_command[p];

        always @(posedge clk) begin
            if (
                rst_n &&
                seq_to_pe_command_valid[p] &&
                pe_to_seq_command_ready[p] &&
                observed_pe_command.op == PE_NEXT &&
                result_write_pending
            )
                $fatal(
                    1,
                    "NEXT issued before shared-memory write acknowledgement"
                );
        end

    end

    assign rs_to_seq_start_ready =
        rs_start_ready_ungated &&
        rs_start_path_enabled;

    assign rs_to_seq_result_valid =
        rs_result_valid_ungated &&
        rs_result_path_enabled;

    reduce_sigmoid #(
        .N_PE(P)
    ) rs (
        .clk_i(clk),
        .rst_n_i(compute_rst_n),

        .pe_sums_i(pe_to_reduce_sum),

        .seq_bias_i(seq_to_rs_bias),
        .seq_start_i(
            seq_to_rs_start && rs_start_path_enabled
        ),
        .seq_start_ready_o(rs_start_ready_ungated),

        .seq_result_data_o(rs_to_seq_result_data),
        .seq_result_status_o(rs_to_seq_result_status),
        .seq_result_valid_o(rs_result_valid_ungated),
        .seq_result_ready_i(
            seq_to_rs_result_ready &&
            rs_result_path_enabled
        )
    );

    // Проверки протоколов и порядка выполнения.
    always @(posedge clk) begin
        cycle_count <= cycle_count + 1;

        if (!rst_n) begin
            io_response_stalled_reg <= 0;
        end else begin

            if (
                io_response_stalled_reg &&
                (
                    !seq_to_io_response_valid ||
                    seq_to_io_response !== held_io_response
                )
            )
                $fatal(1, "Response changed while stalled");

            io_response_stalled_reg <=
                seq_to_io_response_valid &&
                !io_to_seq_response_ready;

            held_io_response <= seq_to_io_response;

            if (
                seq_to_rs_start &&
                rs_to_seq_start_ready &&
                !(&pe_to_seq_layer_done)
            )
                $fatal(1, "Reduction before PE barrier");

            if (
                seq_to_rs_start &&
                !(&pe_sum_valid_for_seq)
            )
                $fatal(
                    1,
                    "RS start before all current sums are valid"
                );

            if (
                inference_in_progress &&
                rs_to_seq_result_valid &&
                seq_to_rs_result_ready
            ) begin
                result_write_pending <= 1;
                accepted_neuron_result_count <=
                    accepted_neuron_result_count + 1;
            end

            if (
                seq_to_shared_mem_request_valid &&
                shared_mem_to_seq_request_ready
            ) begin
                last_memory_request_was_write <=
                    seq_to_shared_mem_request.write;

                last_memory_request_address <=
                    seq_to_shared_mem_request.addr;
            end

            if (
                shared_mem_to_seq_response_valid &&
                seq_to_shared_mem_response_ready &&
                last_memory_request_was_write &&
                result_write_pending &&
                (
                    (
                        last_memory_request_address >= 32'h3000 &&
                        last_memory_request_address < 32'h30C4
                    ) ||
                    (
                        last_memory_request_address >= 32'h3100 &&
                        last_memory_request_address < 32'h3130
                    )
                )
            )
                result_write_pending <= 0;
        end
    end

    task automatic require_file(
        input string filename
    );
        integer fd;

        fd = $fopen(filename, "r");

        if (fd == 0)
            $fatal(
                1,
                "Cannot open %s in simulator working directory",
                filename
            );

        $fclose(fd);
    endtask

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

    task automatic restore_model_image;
        for (int i = 0; i < 663; i++)
            model_image_words[i] = original_model_words[i];
    endtask

    task automatic update_model_crc;
        model_crc_work = 32'hFFFFFFFF;
        model_image_words[6] = 0;

        for (
            int i = 0;
            i < int'(model_image_words[5] >> 2);
            i++
        ) begin
            for (int b = 0; b < 4; b++)
                model_crc_work = crc_byte(
                    model_crc_work,
                    model_image_words[i][8*b +: 8]
                );
        end

        model_image_words[6] =
            model_crc_work ^ 32'hFFFFFFFF;
    endtask

    task automatic send_io_transfer(
        input io_to_seq_t d
    );
        @(negedge clk);

        io_to_seq_request = d;
        io_to_seq_request_valid = 1;

        do @(posedge clk);
        while (!seq_to_io_request_ready);

        @(negedge clk);
        io_to_seq_request_valid = 0;
    endtask

    task automatic send_command_header(
        input logic [7:0] cmd,
        input logic [31:0] arg,
        input logic [15:0] len
    );
        io_to_seq_t d;

        request_sequence = request_sequence + 1;

        d = '0;
        d.is_header = 1;
        d.seq = 16'(request_sequence);
        d.cmd = cmd;

        if (
            cmd == CMD_CONFIG_BEGIN ||
            cmd == CMD_INPUT_BEGIN
        )
            d.image_size = arg;
        else
            d.offset = arg;

        d.data_length = len;

        send_io_transfer(d);
    endtask

    task automatic expect_io_response(
        input logic [7:0] status,
        input logic [31:0] value
    );
        wait (seq_to_io_response_valid);
        @(negedge clk);

        if (
            seq_to_io_response.seq !== 16'(request_sequence) ||
            seq_to_io_response.status !== status ||
            seq_to_io_response.next_offset !== value
        ) begin
            $fatal(
                1,
                "Reply seq%0d: status=%h expected=%h arg=%h expected=%h state=%0d",
                request_sequence,
                seq_to_io_response.status,
                status,
                seq_to_io_response.next_offset,
                value,
                dut.state
            );
        end

        // Получатель намеренно задерживает ready.
        repeat (5) @(negedge clk);

        io_to_seq_response_ready = 1;
        @(negedge clk);

        io_to_seq_response_ready = 0;
        checked_response_count = checked_response_count + 1;
    endtask

    task automatic send_data_chunk(
        input logic [7:0] cmd,
        input integer offset,
        input integer words,
        input logic from_input
    );
        io_to_seq_t d;

        send_command_header(
            cmd,
            32'(offset),
            16'(words*4)
        );

        for (int i = 0; i < words; i++) begin
            d = '0;

            d.word = from_input
                ? input_vector_words[offset/4+i]
                : model_image_words[offset/4+i];

            d.keep = 4'hF;
            d.last = (i == words-1);

            send_io_transfer(d);
        end
    endtask

    task automatic upload_model(
        input logic [7:0] status
    );
        send_command_header(
            CMD_CONFIG_BEGIN,
            model_image_words[5],
            0
        );

        expect_io_response(STATUS_OK, 0);

        for (
            int offset = 0;
            offset < int'(model_image_words[5] >> 2);
            offset += 64
        ) begin
            send_data_chunk(
                CMD_CONFIG_DATA,
                offset*4,
                (int'(model_image_words[5] >> 2)-offset < 64)
                    ? int'(model_image_words[5] >> 2)-offset
                    : 64,
                0
            );

            expect_io_response(
                STATUS_OK,
                32'(
                    (
                        (offset+64 < int'(model_image_words[5] >> 2))
                            ? offset+64
                            : int'(model_image_words[5] >> 2)
                    ) * 4
                )
            );
        end

        send_command_header(CMD_CONFIG_END, 0, 0);

        expect_io_response(
            status,
            (status == STATUS_OK)
                ? model_image_words[5]
                : 0
        );
    endtask

    task automatic upload_input_vector(
        input logic [7:0] status
    );
        send_command_header(
            CMD_INPUT_BEGIN,
            {16'd0, model_image_words[2][15:0]} << 2,
            0
        );

        expect_io_response(STATUS_OK, 0);

        send_data_chunk(
            CMD_INPUT_DATA,
            0,
            int'(model_image_words[2][15:0]),
            1
        );

        expect_io_response(
            STATUS_OK,
            {16'd0, model_image_words[2][15:0]} << 2
        );

        send_command_header(CMD_INPUT_END, 0, 0);

        expect_io_response(
            status,
            (status == STATUS_OK)
                ? ({16'd0, model_image_words[2][15:0]} << 2)
                : 0
        );
    endtask

    task automatic run_and_check_network;
        inference_in_progress = 1;
        accepted_neuron_result_count = 0;

        send_command_header(CMD_RUN, 0, 0);
        expect_io_response(STATUS_OK, 32'h3000);

        inference_in_progress = 0;

        if (accepted_neuron_result_count != 15)
            $fatal(
                1,
                "Wrong number of neuron iterations: %0d",
                accepted_neuron_result_count
            );

        for (int i = 0; i < 12; i++) begin
            send_command_header(
                CMD_READ_WORD,
                32'h3100 + 32'(4*i),
                0
            );

            expect_io_response(
                STATUS_OK,
                expected_output_words[i]
            );
        end

        for (int i = 0; i < 3; i++) begin
            send_command_header(
                CMD_READ_WORD,
                32'h3000 + 32'(4*i),
                0
            );

            expect_io_response(
                STATUS_OK,
                expected_output_words[12+i]
            );
        end

        send_command_header(CMD_READ_WORD, 32'h3200, 0);
        expect_io_response(STATUS_OK, 1);

        send_command_header(CMD_READ_WORD, 32'h3204, 0);
        expect_io_response(STATUS_OK, 3);

        send_command_header(CMD_READ_WORD, 32'h3208, 0);
        expect_io_response(STATUS_OK, 32'h3000);

        send_command_header(CMD_READ_WORD, 32'h320C, 0);
        expect_io_response(STATUS_OK, 0);
    endtask

    initial begin
        io_to_seq_t malformed;

        io_to_seq_request = '0;

        require_file("test_model.mem");
        require_file("test_input.mem");
        require_file($sformatf("test_expected_p%0d.mem", P));
        require_file("sigmoid_lut.mem");

        $readmemh(
            "test_model.mem",
            original_model_words
        );

        $readmemh(
            "test_input.mem",
            input_vector_words
        );

        $readmemh(
            $sformatf("test_expected_p%0d.mem", P),
            expected_output_words
        );

        for (int i = 0; i < 663; i++)
            if ((^original_model_words[i]) === 1'bx)
                $fatal(1, "Missing model word %0d", i);

        for (int i = 0; i < 49; i++)
            if ((^input_vector_words[i]) === 1'bx)
                $fatal(1, "Missing input word %0d", i);

        for (int i = 0; i < 15; i++)
            if ((^expected_output_words[i]) === 1'bx)
                $fatal(1, "Missing expected word %0d", i);

        repeat (5) @(negedge clk);
        rst_n = 1;

        // RUN до загрузки модели.
        send_command_header(CMD_RUN, 0, 0);
        expect_io_response(STATUS_BAD_STATE, 0);

        send_command_header(CMD_CONFIG_BEGIN, 31, 0);
        expect_io_response(STATUS_BAD_SIZE, 0);

        restore_model_image();

        // DATA до CONFIG_BEGIN.
        send_data_chunk(CMD_CONFIG_DATA, 0, 2, 0);
        expect_io_response(STATUS_BAD_STATE, 0);

        send_command_header(CMD_CONFIG_BEGIN, 2652, 0);
        expect_io_response(STATUS_OK, 0);

        // Неверное смещение.
        send_data_chunk(CMD_CONFIG_DATA, 4, 1, 0);
        expect_io_response(STATUS_BAD_OFFSET, 0);

        // Неполная маска keep.
        send_command_header(CMD_CONFIG_DATA, 0, 8);

        malformed = '0;
        malformed.word = model_image_words[0];
        malformed.keep = 3;
        send_io_transfer(malformed);

        malformed.word = model_image_words[1];
        malformed.keep = 15;
        malformed.last = 1;
        send_io_transfer(malformed);

        expect_io_response(STATUS_BAD_STREAM, 0);

        // Ошибка памяти и дочитывание остатка пакета.
        inject_write_error = 1;

        send_data_chunk(CMD_CONFIG_DATA, 0, 4, 0);
        expect_io_response(STATUS_MEMORY_ERROR, 0);

        inject_write_error = 0;

        send_data_chunk(CMD_CONFIG_DATA, 0, 2, 0);
        expect_io_response(STATUS_OK, 8);

        // Образ ещё не загружен полностью.
        send_command_header(CMD_CONFIG_END, 0, 0);
        expect_io_response(STATUS_BAD_STATE, 8);

        $display(
            "PASS chunk state/offset/stream checks and memory-error payload drain"
        );

        // Ошибка CRC всего образа.
        restore_model_image();
        model_image_words[100] = model_image_words[100] ^ 1;

        upload_model(STATUS_MODEL_CRC);

        if (model_valid)
            $fatal(1, "Bad CRC became valid");

        $display("PASS CRC failure and invalid initial state");

        // Ошибка дескриптора при правильном CRC.
        restore_model_image();
        model_image_words[10] = 32'h3000;
        update_model_crc();

        upload_model(STATUS_BAD_MODEL);

        if (model_valid)
            $fatal(1, "Bad descriptor became valid");

        // Корректная модель.
        restore_model_image();
        upload_model(STATUS_OK);

        if (!model_valid)
            $fatal(1, "Model not ready");

        $display(
            "PASS full model validation and weight distribution"
        );

        // Неверный размер входного вектора.
        send_command_header(CMD_INPUT_BEGIN, 192, 0);
        expect_io_response(STATUS_BAD_SIZE, 0);

        // NaN во входном векторе.
        input_vector_words[0] = 32'h7FC00000;
        upload_input_vector(STATUS_BAD_STREAM);

        // Первый успешный запуск.
        $readmemh("test_input.mem", input_vector_words);
        upload_input_vector(STATUS_OK);
        run_and_check_network();

        // Повтор RUN без нового входа запрещён.
        send_command_header(CMD_RUN, 0, 0);
        expect_io_response(STATUS_BAD_STATE, 0);

        send_command_header(CMD_READ_WORD, 1, 0);
        expect_io_response(STATUS_BAD_OFFSET, 0);

        // Второй запуск без повторной загрузки весов.
        upload_input_vector(STATUS_OK);
        run_and_check_network();

        $display(
            "PASS two full inferences, argmax and memory/PE/RS/IO backpressure"
        );

        // Переполнение арифметики PE.
        restore_model_image();
        model_image_words[24] = 32'h7F7FFFFF;

        update_model_crc();
        upload_model(STATUS_OK);

        input_vector_words[0] = 32'h40000000;
        upload_input_vector(STATUS_OK);

        send_command_header(CMD_RUN, 0, 0);
        expect_io_response(STATUS_PE_ERROR, 0);

        if (!model_valid)
            $fatal(
                1,
                "Runtime failure discarded verified model"
            );

        send_command_header(CMD_READ_WORD, 32'h3000, 0);
        expect_io_response(STATUS_BAD_STATE, 0);

        // Переполнение reduce после конечных сумм PE.
        model_image_words[612] = 32'h7F7FFFFF;

        update_model_crc();
        upload_model(STATUS_OK);

        for (int i = 0; i < 49; i++)
            input_vector_words[i] = 0;

        input_vector_words[0] = 32'h3F800000;
        upload_input_vector(STATUS_OK);

        send_command_header(CMD_RUN, 0, 0);
        expect_io_response(STATUS_COMPUTE_ERROR, 0);

        // Восстановление после ошибок.
        restore_model_image();
        upload_model(STATUS_OK);

        $readmemh("test_input.mem", input_vector_words);
        upload_input_vector(STATUS_OK);
        run_and_check_network();

        // Модель с одним слоем: 49 -> 12.
        restore_model_image();

        model_image_words[1] = 32'h00010001;
        model_image_words[2] = 32'h000C0031;

        update_model_crc();
        upload_model(STATUS_OK);
        upload_input_vector(STATUS_OK);

        inference_in_progress = 1;
        accepted_neuron_result_count = 0;

        send_command_header(CMD_RUN, 0, 0);
        expect_io_response(STATUS_OK, 32'h3100);

        inference_in_progress = 0;

        if (accepted_neuron_result_count != 12)
            $fatal(1, "Single-layer iteration count");

        for (int i = 0; i < 12; i++) begin
            send_command_header(
                CMD_READ_WORD,
                32'h3100 + 32'(4*i),
                0
            );

            expect_io_response(
                STATUS_OK,
                expected_output_words[i]
            );
        end

        // Восемь слоёв 1 -> 1.
        // Все веса и bias равны нулю.
        for (int i = 0; i < 663; i++)
            model_image_words[i] = 0;

        model_image_words[0] = 32'h32504E4E;
        model_image_words[1] = 32'h00080001;
        model_image_words[2] = 32'h00010001;

        model_image_words[3] = 32'h00000101;
        model_image_words[4] = 32;
        model_image_words[5] = 352;

        for (int l = 0; l < 8; l++) begin
            model_image_words[8+8*l] = 32'h00010101;
            model_image_words[9+8*l] = 32'h00010001;

            model_image_words[10+8*l] = 32'(288 + 8*l);
            model_image_words[11+8*l] = 32'(292 + 8*l);

            model_image_words[12+8*l] =
                (l % 2 == 0) ? 32'h3000 : 32'h3100;

            model_image_words[13+8*l] =
                (l % 2 == 0) ? 32'h3100 : 32'h3000;

            model_image_words[14+8*l] = 4;
        end

        update_model_crc();
        upload_model(STATUS_OK);

        input_vector_words[0] = 32'h3F800000;
        upload_input_vector(STATUS_OK);

        inference_in_progress = 1;
        accepted_neuron_result_count = 0;

        send_command_header(CMD_RUN, 0, 0);
        expect_io_response(STATUS_OK, 32'h3000);

        inference_in_progress = 0;

        if (accepted_neuron_result_count != 8)
            $fatal(1, "Maximum-layer iteration count");

        send_command_header(CMD_READ_WORD, 32'h3000, 0);
        expect_io_response(STATUS_OK, 32'h3F000000);

        // Девять слоёв должны быть отвергнуты.
        model_image_words[1] = 32'h00090001;
        update_model_crc();

        upload_model(STATUS_BAD_MODEL);

        $display(
            "PASS single layer, eight layers, idle PEs and excessive layer rejection"
        );

        $display(
            "ALL SEQUENCER TESTS PASSED P=%0d, %0d responses",
            P,
            checked_response_count
        );

        $finish;
    end

    initial begin
        #50ms;
        $fatal(1, "Global timeout, state=%0d", dut.state);
    end

endmodule