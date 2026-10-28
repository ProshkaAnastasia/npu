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

    logic rst = 0;

    io_to_seq_t host_data;

    logic host_valid = 0;
    logic host_ready;

    logic reply_valid;
    logic reply_ready = 0;

    seq_to_io_t response;
    seq_to_io_t held_response;

    wire mem_req_t mreq;
    wire mem_rsp_t mrsp;
    wire mem_rsp_t raw_mrsp;

    logic inject_write_error = 0;
    logic last_mem_write = 0;
    logic [31:0] last_mem_addr = 0;

    assign mrsp.value = raw_mrsp.value;

    assign mrsp.error =
        raw_mrsp.error ||
        (
            inject_write_error &&
            last_mem_write &&
            last_mem_addr == 0
        );

    wire mqv;
    wire mqr;
    wire mrv;
    wire mrr;

    wire raw_mqr;
    wire raw_mrv;
    wire raw_mrr;

    wire seq_to_pe_t pcmd [0:P-1];

    wire [P-1:0] pv;
    wire [P-1:0] pr;
    wire [P-1:0] raw_pr;

    wire [P-1:0] pdone;
    wire [P-1:0] pov;
    wire [P-1:0] por;

    wire pe_status_t pstatus [0:P-1];
    wire pe_to_rs_t pout [0:P-1];

    wire seq_to_rs_t rcmd;
    wire rs_to_seq_t rresult;

    wire rs_start;
    wire rs_ready;
    wire rs_valid;
    wire rs_accept;

    wire raw_rs_ready;
    wire raw_rs_valid;

    wire compute_reset;
    wire model_valid;
    wire busy;

    integer cycle = 0;
    integer sequence_id = 0;
    integer responses = 0;

    logic held = 0;

    logic [31:0] base_model [0:662];
    logic [31:0] model [0:662];
    logic [31:0] inputs [0:48];
    logic [31:0] golden [0:14];

    logic [31:0] crc_temp;

    logic run_active = 0;
    logic activation_pending = 0;

    integer rs_count = 0;

    // Искусственные задержки интерфейсов.
    wire mem_gate = (cycle % 5 != 1);
    wire rsp_gate = (cycle % 7 != 2);
    wire rs_gate = (cycle % 4 != 1);
    wire rs_out_gate = (cycle % 6 != 3);

    sequencer #(
        .N_PE(P)
    ) dut (
        .clk_i(clk),
        .rst_n_i(rst),

        .io_to_seq_data_i(host_data),
        .io_to_seq_valid_i(host_valid),
        .io_to_seq_ready_o(host_ready),

        .seq_to_io_data_o(response),
        .seq_to_io_valid_o(reply_valid),
        .seq_to_io_ready_i(reply_ready),

        .seq_to_mem_data_o(mreq),
        .seq_to_mem_valid_o(mqv),
        .seq_to_mem_ready_i(mqr),

        .mem_to_seq_data_i(mrsp),
        .mem_to_seq_valid_i(mrv),
        .mem_to_seq_ready_o(mrr),

        .seq_to_pe_data_o(pcmd),
        .seq_to_pe_valid_o(pv),
        .seq_to_pe_ready_i(pr),

        .pe_done_i(pdone),
        .pe_status_i(pstatus),

        .seq_to_rs_data_o(rcmd),
        .seq_to_rs_start_o(rs_start),
        .seq_to_rs_ready_i(rs_ready),

        .rs_to_seq_data_i(rresult),
        .rs_to_seq_valid_i(rs_valid),
        .rs_to_seq_ready_o(rs_accept),

        .compute_rst_n_o(compute_reset),
        .model_valid_o(model_valid),
        .busy_o(busy)
    );

    assign mqr = raw_mqr && mem_gate;
    assign mrv = raw_mrv && rsp_gate;
    assign raw_mrr = mrr && rsp_gate;

    memory #(
        .MEM_WORDS(4096)
    ) shared_memory (
        .clk_i(clk),
        .rst_n_i(rst),

        .a_req_data_i(mreq),
        .a_req_valid_i(mqv && mem_gate),
        .a_req_ready_o(raw_mqr),

        .a_rsp_data_o(raw_mrsp),
        .a_rsp_valid_o(raw_mrv),
        .a_rsp_ready_i(raw_mrr),

        .b_req_data_i('0),
        .b_req_valid_i(1'b0),
        .b_req_ready_o(),

        .b_rsp_data_o(),
        .b_rsp_valid_o(),
        .b_rsp_ready_i(1'b1)
    );

    for (genvar p = 0; p < P; p++) begin : g_pe

        wire gate = ((cycle+p) % 4 != 0);

        assign pr[p] = raw_pr[p] && gate;

        pe u_pe (
            .clk_i(clk),
            .rst_n_i(compute_reset),

            .seq_to_pe_data_i(pcmd[p]),
            .seq_to_pe_valid_i(pv[p] && gate),
            .seq_to_pe_ready_o(raw_pr[p]),

            .pe_to_reduce_data_o(pout[p]),
            .pe_to_reduce_valid_o(pov[p]),
            .pe_to_reduce_ready_i(por[p]),

            .pe_done_o(pdone[p]),
            .pe_status_o(pstatus[p])
        );

        wire seq_to_pe_t observed_cmd = pcmd[p];

        always @(posedge clk) begin
            if (
                rst &&
                pv[p] &&
                pr[p] &&
                observed_cmd.op == PE_NEXT &&
                activation_pending
            )
                $fatal(
                    1,
                    "NEXT issued before shared-memory write acknowledgement"
                );
        end

    end

    assign rs_ready = raw_rs_ready && rs_gate;
    assign rs_valid = raw_rs_valid && rs_out_gate;

    reduce_sigmoid #(
        .N_PE(P)
    ) rs (
        .clk_i(clk),
        .rst_n_i(compute_reset),

        .seq_to_rs_data_i(rcmd),
        .seq_to_rs_start_i(rs_start && rs_gate),
        .seq_to_rs_ready_o(raw_rs_ready),

        .pe_to_rs_data_i(pout),
        .pe_to_rs_valid_i(pov),
        .pe_to_rs_ready_o(por),

        .rs_to_seq_data_o(rresult),
        .rs_to_seq_valid_o(raw_rs_valid),
        .rs_to_seq_ready_i(rs_accept && rs_out_gate)
    );

    always @(posedge clk) begin
        cycle <= cycle + 1;

        if (!rst) begin
            held <= 0;
        end else begin

            if (
                held &&
                (!reply_valid || response !== held_response)
            )
                $fatal(1, "Response changed while stalled");

            held <= reply_valid && !reply_ready;
            held_response <= response;

            if (rs_start && rs_ready && !(&pdone))
                $fatal(1, "Reduction before PE barrier");

            if (run_active && rs_valid && rs_accept) begin
                activation_pending <= 1;
                rs_count <= rs_count + 1;
            end

            if (mqv && mqr) begin
                last_mem_write <= mreq.write;
                last_mem_addr <= mreq.addr;
            end

            if (
                mrv &&
                mrr &&
                last_mem_write &&
                activation_pending &&
                (
                    (
                        last_mem_addr >= 32'h3000 &&
                        last_mem_addr < 32'h30C4
                    ) ||
                    (
                        last_mem_addr >= 32'h3100 &&
                        last_mem_addr < 32'h3130
                    )
                )
            )
                activation_pending <= 0;
        end
    end

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

    task automatic restore_model;
        for (int i = 0; i < 663; i++)
            model[i] = base_model[i];
    endtask

    task automatic update_crc;
        crc_temp = 32'hFFFFFFFF;
        model[6] = 0;

        for (int i = 0; i < int'(model[5] >> 2); i++) begin
            for (int b = 0; b < 4; b++)
                crc_temp = crc_byte(
                    crc_temp,
                    model[i][8*b +: 8]
                );
        end

        model[6] = crc_temp ^ 32'hFFFFFFFF;
    endtask

    task automatic send_word(input io_to_seq_t d);
        @(negedge clk);

        host_data = d;
        host_valid = 1;

        do @(posedge clk);
        while (!host_ready);

        @(negedge clk);
        host_valid = 0;
    endtask

    task automatic header(
        input logic [7:0] cmd,
        input logic [31:0] arg,
        input logic [15:0] len
    );
        io_to_seq_t d;

        sequence_id = sequence_id + 1;

        d = '0;
        d.is_header = 1;
        d.seq = 16'(sequence_id);
        d.cmd = cmd;

        if (cmd == CMD_CONFIG_BEGIN || cmd == CMD_INPUT_BEGIN)
            d.image_size = arg;
        else
            d.offset = arg;

        d.data_length = len;

        send_word(d);
    endtask

    task automatic expect_reply(
        input logic [7:0] status,
        input logic [31:0] value
    );
        wait (reply_valid);
        @(negedge clk);

        if (
            response.seq !== 16'(sequence_id) ||
            response.status !== status ||
            response.next_offset !== value
        ) begin
            $fatal(
                1,
                "Reply seq%0d: status=%h expected=%h arg=%h expected=%h state=%0d",
                sequence_id,
                response.status,
                status,
                response.next_offset,
                value,
                dut.state
            );
        end

        // Ответ должен оставаться неизменным до ready.
        repeat (5) @(negedge clk);

        reply_ready = 1;
        @(negedge clk);

        reply_ready = 0;
        responses = responses + 1;
    endtask

    task automatic chunk(
        input logic [7:0] cmd,
        input integer offset,
        input integer words,
        input logic from_input
    );
        io_to_seq_t d;

        header(cmd, 32'(offset), 16'(words*4));

        for (int i = 0; i < words; i++) begin
            d = '0;

            d.word = from_input
                ? inputs[offset/4+i]
                : model[offset/4+i];

            d.keep = 4'hF;
            d.last = (i == words-1);

            send_word(d);
        end
    endtask

    task automatic upload(input logic [7:0] status);
        header(CMD_CONFIG_BEGIN, model[5], 0);
        expect_reply(STATUS_OK, 0);

        for (
            int offset = 0;
            offset < int'(model[5] >> 2);
            offset += 64
        ) begin
            chunk(
                CMD_CONFIG_DATA,
                offset*4,
                (int'(model[5] >> 2)-offset < 64)
                    ? int'(model[5] >> 2)-offset
                    : 64,
                0
            );

            expect_reply(
                STATUS_OK,
                32'(
                    (
                        (offset+64 < int'(model[5] >> 2))
                            ? offset+64
                            : int'(model[5] >> 2)
                    ) * 4
                )
            );
        end

        header(CMD_CONFIG_END, 0, 0);

        expect_reply(
            status,
            (status == STATUS_OK) ? model[5] : 0
        );
    endtask

    task automatic input_upload(input logic [7:0] status);
        header(
            CMD_INPUT_BEGIN,
            {16'd0, model[2][15:0]} << 2,
            0
        );

        expect_reply(STATUS_OK, 0);

        chunk(
            CMD_INPUT_DATA,
            0,
            int'(model[2][15:0]),
            1
        );

        expect_reply(
            STATUS_OK,
            {16'd0, model[2][15:0]} << 2
        );

        header(CMD_INPUT_END, 0, 0);

        expect_reply(
            status,
            (status == STATUS_OK)
                ? ({16'd0, model[2][15:0]} << 2)
                : 0
        );
    endtask

    task automatic infer_and_check;
        run_active = 1;
        rs_count = 0;

        header(CMD_RUN, 0, 0);
        expect_reply(STATUS_OK, 32'h3000);

        run_active = 0;

        if (rs_count != 15)
            $fatal(
                1,
                "Wrong number of neuron iterations: %0d",
                rs_count
            );

        // Промежуточный слой: 12 нейронов.
        for (int i = 0; i < 12; i++) begin
            header(CMD_READ_WORD, 32'h3100 + 32'(4*i), 0);
            expect_reply(STATUS_OK, golden[i]);
        end

        // Последний слой: 3 нейрона.
        for (int i = 0; i < 3; i++) begin
            header(CMD_READ_WORD, 32'h3000 + 32'(4*i), 0);
            expect_reply(STATUS_OK, golden[12+i]);
        end

        // Служебная запись.
        header(CMD_READ_WORD, 32'h3200, 0);
        expect_reply(STATUS_OK, 1);

        header(CMD_READ_WORD, 32'h3204, 0);
        expect_reply(STATUS_OK, 3);

        header(CMD_READ_WORD, 32'h3208, 0);
        expect_reply(STATUS_OK, 32'h3000);

        header(CMD_READ_WORD, 32'h320C, 0);
        expect_reply(STATUS_OK, 0);
    endtask

    initial begin
        io_to_seq_t malformed;

        host_data = '0;

        $readmemh("test_model.mem", base_model);
        $readmemh("test_input.mem", inputs);

        $readmemh(
            $sformatf("test_expected_p%0d.mem", P),
            golden
        );

        repeat (5) @(negedge clk);
        rst = 1;

        // RUN до загрузки модели запрещён.
        header(CMD_RUN, 0, 0);
        expect_reply(STATUS_BAD_STATE, 0);

        header(CMD_CONFIG_BEGIN, 31, 0);
        expect_reply(STATUS_BAD_SIZE, 0);

        restore_model();

        // DATA до BEGIN запрещён.
        chunk(CMD_CONFIG_DATA, 0, 2, 0);
        expect_reply(STATUS_BAD_STATE, 0);

        header(CMD_CONFIG_BEGIN, 2652, 0);
        expect_reply(STATUS_OK, 0);

        // Неверное смещение.
        chunk(CMD_CONFIG_DATA, 4, 1, 0);
        expect_reply(STATUS_BAD_OFFSET, 0);

        // Некорректный keep.
        header(CMD_CONFIG_DATA, 0, 8);

        malformed = '0;
        malformed.word = model[0];
        malformed.keep = 3;
        send_word(malformed);

        malformed.word = model[1];
        malformed.keep = 15;
        malformed.last = 1;
        send_word(malformed);

        expect_reply(STATUS_BAD_STREAM, 0);

        // Ошибка записи памяти: остаток пакета нужно дочитать.
        inject_write_error = 1;

        chunk(CMD_CONFIG_DATA, 0, 4, 0);
        expect_reply(STATUS_MEMORY_ERROR, 0);

        inject_write_error = 0;

        // Повторная передача с того же смещения.
        chunk(CMD_CONFIG_DATA, 0, 2, 0);
        expect_reply(STATUS_OK, 8);

        // Образ ещё загружен не полностью.
        header(CMD_CONFIG_END, 0, 0);
        expect_reply(STATUS_BAD_STATE, 8);

        $display(
            "PASS chunk state/offset/stream checks and memory-error payload drain"
        );

        // Неверный CRC всего образа.
        restore_model();
        model[100] = model[100] ^ 1;

        upload(STATUS_MODEL_CRC);

        if (model_valid)
            $fatal(1, "Bad CRC became valid");

        $display("PASS CRC failure and invalid initial state");

        // Неверный дескриптор с корректным CRC.
        restore_model();
        model[10] = 32'h3000;
        update_crc();

        upload(STATUS_BAD_MODEL);

        if (model_valid)
            $fatal(1, "Bad descriptor became valid");

        // Корректная модель.
        restore_model();
        upload(STATUS_OK);

        if (!model_valid)
            $fatal(1, "Model not ready");

        $display("PASS full model validation and weight distribution");

        // Неверный размер входного вектора.
        header(CMD_INPUT_BEGIN, 192, 0);
        expect_reply(STATUS_BAD_SIZE, 0);

        // NaN во входном векторе.
        inputs[0] = 32'h7FC00000;
        input_upload(STATUS_BAD_STREAM);

        // Первый успешный запуск.
        $readmemh("test_input.mem", inputs);
        input_upload(STATUS_OK);
        infer_and_check();

        // Перед следующим RUN нужен новый входной вектор.
        header(CMD_RUN, 0, 0);
        expect_reply(STATUS_BAD_STATE, 0);

        header(CMD_READ_WORD, 1, 0);
        expect_reply(STATUS_BAD_OFFSET, 0);

        // Второй успешный запуск без перезагрузки весов.
        input_upload(STATUS_OK);
        infer_and_check();

        $display(
            "PASS two full inferences, argmax and memory/PE/RS/IO backpressure"
        );

        // Переполнение в PE.
        restore_model();
        model[24] = 32'h7F7FFFFF;
        update_crc();
        upload(STATUS_OK);

        inputs[0] = 32'h40000000;
        input_upload(STATUS_OK);

        header(CMD_RUN, 0, 0);
        expect_reply(STATUS_PE_ERROR, 0);

        if (!model_valid)
            $fatal(
                1,
                "Runtime failure discarded verified model"
            );

        header(CMD_READ_WORD, 32'h3000, 0);
        expect_reply(STATUS_BAD_STATE, 0);

        // Переполнение reduce при конечных частичных суммах.
        model[612] = 32'h7F7FFFFF;
        update_crc();
        upload(STATUS_OK);

        for (int i = 0; i < 49; i++)
            inputs[i] = 0;

        inputs[0] = 32'h3F800000;
        input_upload(STATUS_OK);

        header(CMD_RUN, 0, 0);
        expect_reply(STATUS_COMPUTE_ERROR, 0);

        // Восстановление после ошибок.
        restore_model();
        upload(STATUS_OK);

        $readmemh("test_input.mem", inputs);
        input_upload(STATUS_OK);
        infer_and_check();

        // Модель с одним слоем 49 -> 12.
        restore_model();

        model[1] = 32'h00010001;
        model[2] = 32'h000C0031;

        update_crc();
        upload(STATUS_OK);
        input_upload(STATUS_OK);

        run_active = 1;
        rs_count = 0;

        header(CMD_RUN, 0, 0);
        expect_reply(STATUS_OK, 32'h3100);

        run_active = 0;

        if (rs_count != 12)
            $fatal(1, "Single-layer iteration count");

        for (int i = 0; i < 12; i++) begin
            header(CMD_READ_WORD, 32'h3100 + 32'(4*i), 0);
            expect_reply(STATUS_OK, golden[i]);
        end

        // Модель из восьми слоёв 1 -> 1.
        // Веса и bias равны нулю, результат sigmoid(0) = 0.5.
        for (int i = 0; i < 663; i++)
            model[i] = 0;

        model[0] = 32'h32504E4E;
        model[1] = 32'h00080001;
        model[2] = 32'h00010001;

        model[3] = 32'h00000101;
        model[4] = 32;
        model[5] = 352;

        for (int l = 0; l < 8; l++) begin
            model[8+8*l] = 32'h00010101;
            model[9+8*l] = 32'h00010001;

            model[10+8*l] = 32'(288 + 8*l);
            model[11+8*l] = 32'(292 + 8*l);

            model[12+8*l] =
                (l % 2 == 0) ? 32'h3000 : 32'h3100;

            model[13+8*l] =
                (l % 2 == 0) ? 32'h3100 : 32'h3000;

            model[14+8*l] = 4;
        end

        update_crc();
        upload(STATUS_OK);

        inputs[0] = 32'h3F800000;
        input_upload(STATUS_OK);

        run_active = 1;
        rs_count = 0;

        header(CMD_RUN, 0, 0);
        expect_reply(STATUS_OK, 32'h3000);

        run_active = 0;

        if (rs_count != 8)
            $fatal(1, "Maximum-layer iteration count");

        header(CMD_READ_WORD, 32'h3000, 0);
        expect_reply(STATUS_OK, 32'h3F000000);

        // Девять слоёв должны быть отвергнуты.
        model[1] = 32'h00090001;
        update_crc();

        upload(STATUS_BAD_MODEL);

        $display(
            "PASS single layer, eight layers, idle PEs and excessive layer rejection"
        );

        $display(
            "ALL SEQUENCER TESTS PASSED P=%0d, %0d responses",
            P,
            responses
        );

        $finish;
    end

    initial begin
        #50ms;
        $fatal(1, "Global timeout, state=%0d", dut.state);
    end

endmodule