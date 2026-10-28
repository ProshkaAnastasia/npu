`timescale 1ns/1ps

module sequencer #(
    parameter int N_PE = 6,
    parameter int MAX_LAYERS = 8,
    parameter int MAX_NEURONS = 49,

    parameter int LOCAL_WORDS = 2048,
    parameter int MAX_WEIGHT_WORDS = 1900,
    parameter int SHARED_BYTES = 16384,

    parameter logic [31:0] MODEL_LIMIT = 32'h3000,
    parameter logic [31:0] RESULT_RECORD = 32'h3200
) (
    input logic clk_i,
    input logic rst_n_i,

    // IO -> секвенсор.
    input io_types_pkg::io_to_seq_t io_to_seq_data_i,
    input logic io_to_seq_valid_i,
    output logic io_to_seq_ready_o,

    // Секвенсор -> IO.
    output io_types_pkg::seq_to_io_t seq_to_io_data_o,
    output logic seq_to_io_valid_o,
    input logic seq_to_io_ready_i,

    // Секвенсор -> shared memory: запрос.
    output memory_types_pkg::mem_req_t seq_to_mem_data_o,
    output logic seq_to_mem_valid_o,
    input logic seq_to_mem_ready_i,

    // Shared memory -> секвенсор: ответ.
    input memory_types_pkg::mem_rsp_t mem_to_seq_data_i,
    input logic mem_to_seq_valid_i,
    output logic mem_to_seq_ready_o,

    // Секвенсор -> PE: команды.
    output pe_types_pkg::seq_to_pe_t
        seq_to_pe_command_o [0:N_PE-1],

    output logic [N_PE-1:0] seq_to_pe_command_valid_o,
    input logic [N_PE-1:0] pe_to_seq_command_ready_i,

    // PE -> секвенсор: состояние вычислений и текущей суммы.
    input logic [N_PE-1:0] pe_to_seq_layer_done_i,
    input logic [N_PE-1:0] pe_to_seq_sum_valid_i,

    input pe_types_pkg::pe_status_t
        pe_to_seq_status_i [0:N_PE-1],

    // Секвенсор -> reduce_sigmoid: bias и запуск.
    output logic [31:0] seq_to_rs_bias_o,
    output logic seq_to_rs_start_o,
    input logic rs_to_seq_start_ready_i,

    // Reduce_sigmoid -> секвенсор: результат.
    input logic [31:0] rs_to_seq_result_data_i,
    input rs_types_pkg::rs_status_t rs_to_seq_result_status_i,
    input logic rs_to_seq_result_valid_i,
    output logic seq_to_rs_result_ready_o,

    // Сброс вычислительных блоков.
    output logic compute_rst_n_o,

    output logic model_valid_o,
    output logic busy_o
);

    import io_types_pkg::*;
    import pe_types_pkg::*;
    import memory_types_pkg::*;
    import rs_types_pkg::*;

    typedef enum logic [6:0] {
        HW_RESET,
        IDLE,
        REPLY,

        RX_WORD,
        RX_AFTER_WRITE,
        RX_NEXT,

        MEM_REQ,
        MEM_WAIT,
        PE_SEND,

        HDR_READ,
        HDR_GOT,
        HDR_CHECK,

        CRC_READ,
        CRC_GOT,

        DESC_READ,
        DESC_GOT,
        DESC_CHECK,

        WEIGHTS_BEGIN,
        WEIGHT_READ,
        WEIGHT_GOT,
        WEIGHT_NEXT,

        BIAS_VALIDATE_BEGIN,
        BIAS_VALIDATE_READ,
        BIAS_VALIDATE_GOT,
        MODEL_READY,

        INPUT_CHECK_READ,
        INPUT_CHECK_GOT,
        READ_REPLY,

        RUN_BEGIN,
        LAYER_BEGIN,
        CFG_BUILD,
        CFG_NEXT,

        X_READ,
        X_GOT,
        X_NEXT,

        START_PES,
        WAIT_PES,

        BIAS_READ,
        BIAS_GOT,

        RS_START,
        RS_WAIT,
        RESULT_STORED,

        NEXT_PES,
        NEXT_NEURON,

        WRITE_RECORD,
        RECORD_NEXT
    } state_t;

    state_t state;
    state_t mem_return;
    state_t pe_return;
    state_t reset_return;

    seq_to_io_t response;
    mem_req_t mem_command;
    seq_to_pe_t pe_command;

    logic [31:0] memory_value;

    // PE, которые ещё не приняли текущую команду.
    logic [N_PE-1:0] pending_pe;

    wire [N_PE-1:0] pe_fault;

    for (genvar p = 0; p < N_PE; p++) begin : g_pe
        assign seq_to_pe_command_o[p] = pe_command;

        assign pe_fault[p] =
            (pe_to_seq_status_i[p] != PE_OK);
    end

    logic [2:0] reset_count;
    logic compute_reset_reg;

    logic model_valid;
    logic input_valid;
    logic results_valid;

    logic model_loading;
    logic input_loading;

    logic [31:0] image_size;
    logic [31:0] image_loaded;
    logic [31:0] input_loaded;

    logic [15:0] model_inputs;
    logic [15:0] model_outputs;
    logic [15:0] layer_count;

    logic [31:0] descriptor_offset;
    logic [31:0] expected_crc;
    logic [31:0] crc_reg;

    logic [31:0] header [0:7];
    logic [31:0] descriptor [0:7];

    logic [15:0] layer_n [0:MAX_LAYERS-1];
    logic [15:0] layer_m [0:MAX_LAYERS-1];

    logic [31:0] weight_offset [0:MAX_LAYERS-1];
    logic [31:0] bias_offset [0:MAX_LAYERS-1];

    logic [31:0] input_addr [0:MAX_LAYERS-1];
    logic [31:0] output_addr [0:MAX_LAYERS-1];

    logic [31:0] local_w_base [0:MAX_LAYERS-1];
    logic [31:0] weight_count [0:MAX_LAYERS-1];

    logic [31:0] quota [0:MAX_LAYERS-1];
    logic [31:0] remainder [0:MAX_LAYERS-1];

    logic [15:0] i_step [0:MAX_LAYERS-1];
    logic [15:0] j_step [0:MAX_LAYERS-1];

    logic [31:0] reserved_weights;
    logic [31:0] local_x_base;
    logic [31:0] local_s_base;

    integer layer_index;
    integer word_index;
    integer pe_index;
    integer local_index;
    integer neuron_index;
    integer input_index;
    integer record_index;

    logic [15:0] first_i;
    logic [15:0] first_j;

    logic [31:0] best_value;
    logic [31:0] best_class;
    logic [31:0] bias_reg;

    logic rx_is_input;

    logic [31:0] rx_offset;
    logic [31:0] rx_length;
    logic [31:0] rx_remaining;
    logic [31:0] rx_received;
    logic [31:0] rx_word;

    logic [7:0] rx_error;

    // Комбинационные значения для проверки дескриптора.
    logic [15:0] dn;
    logic [15:0] dm;

    logic [31:0] dtotal;
    logic [31:0] dreserve;

    logic [63:0] dend;
    logic [63:0] w_end;
    logic [63:0] b_end;
    logic [63:0] in_end;
    logic [63:0] out_end;

    logic desc_ok;

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

    function automatic logic [31:0] crc_word(
        input logic [31:0] c0,
        input logic [31:0] w
    );
        logic [31:0] c;

        c = c0;

        for (int k = 0; k < 4; k++)
            c = crc_byte(c, w[8*k +: 8]);

        return c;
    endfunction

    function automatic logic overlap(
        input logic [63:0] a,
        input logic [63:0] ae,
        input logic [63:0] b,
        input logic [63:0] be
    );
        return a < be && b < ae;
    endfunction

    task automatic read_memory(
        input logic [31:0] addr,
        input state_t next_state
    );
        mem_command.write <= 1'b0;
        mem_command.addr <= addr;
        mem_command.value <= '0;

        mem_return <= next_state;
        state <= MEM_REQ;
    endtask

    task automatic write_memory(
        input logic [31:0] addr,
        input logic [31:0] value,
        input state_t next_state
    );
        mem_command.write <= 1'b1;
        mem_command.addr <= addr;
        mem_command.value <= value;

        mem_return <= next_state;
        state <= MEM_REQ;
    endtask

    task automatic reply(
        input logic [7:0] status,
        input logic [31:0] value
    );
        response.status <= status;
        response.next_offset <= value;
        state <= REPLY;
    endtask

    task automatic fail(
        input logic [7:0] status,
        input logic invalidate_model
    );
        response.status <= status;
        response.next_offset <= '0;

        if (invalidate_model)
            model_valid <= 1'b0;

        input_valid <= 1'b0;
        results_valid <= 1'b0;
        model_loading <= 1'b0;
        input_loading <= 1'b0;

        compute_reset_reg <= 1'b0;
        reset_count <= 3;
        reset_return <= REPLY;
        state <= HW_RESET;
    endtask

    task automatic send_pe(
        input logic [N_PE-1:0] mask,
        input state_t next_state
    );
        pending_pe <= mask;
        pe_return <= next_state;
        state <= PE_SEND;
    endtask

    always_comb begin
        io_to_seq_ready_o =
            (state == IDLE || state == RX_WORD);

        seq_to_io_data_o = response;
        seq_to_io_valid_o = (state == REPLY);

        seq_to_mem_data_o = mem_command;
        seq_to_mem_valid_o = (state == MEM_REQ);
        mem_to_seq_ready_o = (state == MEM_WAIT);

        seq_to_pe_command_valid_o =
            (state == PE_SEND) ? pending_pe : '0;

        seq_to_rs_bias_o = bias_reg;

        // Start разрешён только после выставления
        // текущей суммы всеми PE.
        seq_to_rs_start_o =
            (state == RS_START) &&
            (&pe_to_seq_sum_valid_i);

        seq_to_rs_result_ready_o =
            (state == RS_WAIT);

        compute_rst_n_o =
            rst_n_i && compute_reset_reg;

        model_valid_o = model_valid;
        busy_o = (state != IDLE);

        dn = descriptor[1][15:0];
        dm = descriptor[1][31:16];

        dtotal = {16'd0, dn} * {16'd0, dm};

        dreserve =
            (dtotal + 32'(N_PE-1)) / 32'(N_PE);

        dend =
            {32'd0, descriptor_offset} +
            ({48'd0, layer_count} << 5);

        w_end =
            {32'd0, descriptor[2]} +
            ({32'd0, dtotal} << 2);

        b_end =
            {32'd0, descriptor[3]} +
            ({48'd0, dm} << 2);

        in_end =
            {32'd0, descriptor[4]} +
            ({48'd0, dn} << 2);

        out_end =
            {32'd0, descriptor[5]} +
            ({48'd0, dm} << 2);

        desc_ok =
            descriptor[0] == 32'h00010101 &&
            descriptor[7] == 0 &&

            dn != 0 &&
            dm != 0 &&

            int'(dn) <= MAX_NEURONS &&
            int'(dm) <= MAX_NEURONS &&

            descriptor[6] == ({16'd0, dm} << 2) &&

            descriptor[2][1:0] == 0 &&
            descriptor[3][1:0] == 0 &&
            descriptor[4][1:0] == 0 &&
            descriptor[5][1:0] == 0 &&

            {32'd0, descriptor[2]} >= dend &&
            {32'd0, descriptor[3]} >= dend &&

            w_end <= {32'd0, image_size} &&
            b_end <= {32'd0, image_size} &&

            !overlap(
                {32'd0, descriptor[2]}, w_end,
                {32'd0, descriptor[3]}, b_end
            ) &&

            descriptor[4] >= MODEL_LIMIT &&
            descriptor[5] >= MODEL_LIMIT &&

            in_end <= 64'(SHARED_BYTES) &&
            out_end <= 64'(SHARED_BYTES) &&

            !overlap(
                {32'd0, descriptor[4]}, in_end,
                {32'd0, descriptor[5]}, out_end
            ) &&

            !overlap(
                {32'd0, descriptor[4]}, in_end,
                {32'd0, RESULT_RECORD},
                {32'd0, RESULT_RECORD} + 64'd16
            ) &&

            !overlap(
                {32'd0, descriptor[5]}, out_end,
                {32'd0, RESULT_RECORD},
                {32'd0, RESULT_RECORD} + 64'd16
            ) &&

            reserved_weights + dreserve
                <= 32'(MAX_WEIGHT_WORDS) &&

            reserved_weights + dreserve +
                32'(2*MAX_NEURONS)
                <= 32'(LOCAL_WORDS);

        if (layer_index == 0) begin
            desc_ok =
                desc_ok && dn == model_inputs;
        end else if (layer_index < MAX_LAYERS) begin
            desc_ok =
                desc_ok &&
                dn == layer_m[layer_index-1] &&
                descriptor[4] == output_addr[layer_index-1];
        end

        if (layer_index == int'(layer_count)-1)
            desc_ok = desc_ok && dm == model_outputs;
    end

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            state <= HW_RESET;
            reset_return <= IDLE;
            reset_count <= 3;
            compute_reset_reg <= 1'b0;

            mem_return <= IDLE;
            pe_return <= IDLE;

            response <= '0;
            mem_command <= '0;
            pe_command <= '0;
            pending_pe <= '0;
            memory_value <= '0;

            model_valid <= 1'b0;
            input_valid <= 1'b0;
            results_valid <= 1'b0;
            model_loading <= 1'b0;
            input_loading <= 1'b0;

            image_size <= '0;
            image_loaded <= '0;
            input_loaded <= '0;

            model_inputs <= '0;
            model_outputs <= '0;
            layer_count <= '0;
            descriptor_offset <= '0;

            expected_crc <= '0;
            crc_reg <= '0;

            reserved_weights <= '0;
            local_x_base <= '0;
            local_s_base <= '0;

            layer_index <= 0;
            word_index <= 0;
            pe_index <= 0;
            local_index <= 0;
            neuron_index <= 0;
            input_index <= 0;
            record_index <= 0;

            first_i <= '0;
            first_j <= '0;

            best_value <= '0;
            best_class <= '0;
            bias_reg <= '0;

            rx_is_input <= 1'b0;
            rx_offset <= '0;
            rx_length <= '0;
            rx_remaining <= '0;
            rx_received <= '0;
            rx_error <= '0;
            rx_word <= '0;

            for (int j = 0; j < 8; j++) begin
                header[j] <= '0;
                descriptor[j] <= '0;
            end

            for (int j = 0; j < MAX_LAYERS; j++) begin
                layer_n[j] <= '0;
                layer_m[j] <= '0;

                weight_offset[j] <= '0;
                bias_offset[j] <= '0;

                input_addr[j] <= '0;
                output_addr[j] <= '0;

                local_w_base[j] <= '0;
                weight_count[j] <= '0;

                quota[j] <= '0;
                remainder[j] <= '0;

                i_step[j] <= '0;
                j_step[j] <= '0;
            end

        end else begin
            case (state)

                HW_RESET: begin
                    if (reset_count == 1) begin
                        reset_count <= 0;
                        compute_reset_reg <= 1'b1;
                        state <= reset_return;
                    end else begin
                        reset_count <= reset_count - 1'b1;
                    end
                end

                IDLE: begin
                    if (io_to_seq_valid_i && io_to_seq_ready_o) begin
                        response.seq <= io_to_seq_data_i.seq;

                        if (!io_to_seq_data_i.is_header) begin
                            reply(STATUS_BAD_STREAM, 0);
                        end else begin
                            case (io_to_seq_data_i.cmd)

                                CMD_CONFIG_BEGIN: begin
                                    model_valid <= 1'b0;
                                    input_valid <= 1'b0;
                                    results_valid <= 1'b0;

                                    model_loading <= 1'b0;
                                    input_loading <= 1'b0;
                                    image_loaded <= '0;

                                    if (
                                        io_to_seq_data_i.image_size < 32 ||
                                        io_to_seq_data_i.image_size > MODEL_LIMIT ||
                                        io_to_seq_data_i.image_size[1:0] != 0
                                    ) begin
                                        reply(STATUS_BAD_SIZE, 0);
                                    end else begin
                                        image_size <=
                                            io_to_seq_data_i.image_size;

                                        model_loading <= 1'b1;

                                        response.status <= STATUS_OK;
                                        response.next_offset <= '0;

                                        compute_reset_reg <= 1'b0;
                                        reset_count <= 3;
                                        reset_return <= REPLY;
                                        state <= HW_RESET;
                                    end
                                end

                                CMD_INPUT_BEGIN: begin
                                    input_valid <= 1'b0;
                                    results_valid <= 1'b0;
                                    input_loading <= 1'b0;
                                    input_loaded <= '0;

                                    if (!model_valid || model_loading) begin
                                        reply(STATUS_BAD_STATE, 0);

                                    end else if (
                                        io_to_seq_data_i.image_size !=
                                        ({16'd0, model_inputs} << 2)
                                    ) begin
                                        reply(STATUS_BAD_SIZE, 0);

                                    end else begin
                                        input_loading <= 1'b1;
                                        reply(STATUS_OK, 0);
                                    end
                                end

                                CMD_CONFIG_DATA,
                                CMD_INPUT_DATA: begin
                                    rx_is_input <=
                                        (io_to_seq_data_i.cmd == CMD_INPUT_DATA);

                                    rx_offset <= io_to_seq_data_i.offset;

                                    rx_length <= {
                                        16'd0,
                                        io_to_seq_data_i.data_length
                                    };

                                    rx_remaining <= {
                                        16'd0,
                                        io_to_seq_data_i.data_length
                                    };

                                    rx_received <= '0;
                                    rx_error <= STATUS_OK;

                                    if (
                                        io_to_seq_data_i.cmd == CMD_CONFIG_DATA
                                    ) begin
                                        if (!model_loading)
                                            rx_error <= STATUS_BAD_STATE;

                                        else if (
                                            io_to_seq_data_i.offset !=
                                            image_loaded
                                        )
                                            rx_error <= STATUS_BAD_OFFSET;

                                        else if (
                                            {1'b0, io_to_seq_data_i.offset} +
                                            {17'd0, io_to_seq_data_i.data_length}
                                            > {1'b0, image_size}
                                        )
                                            rx_error <= STATUS_BAD_SIZE;

                                    end else begin
                                        if (!input_loading || !model_valid)
                                            rx_error <= STATUS_BAD_STATE;

                                        else if (
                                            io_to_seq_data_i.offset !=
                                            input_loaded
                                        )
                                            rx_error <= STATUS_BAD_OFFSET;

                                        else if (
                                            {1'b0, io_to_seq_data_i.offset} +
                                            {17'd0, io_to_seq_data_i.data_length}
                                            > ({17'd0, model_inputs} << 2)
                                        )
                                            rx_error <= STATUS_BAD_SIZE;
                                    end

                                    if (
                                        io_to_seq_data_i.offset[1:0] != 0 ||
                                        io_to_seq_data_i.data_length[1:0] != 0
                                    )
                                        rx_error <= STATUS_BAD_SIZE;

                                    if (io_to_seq_data_i.data_length == 0)
                                        reply(STATUS_BAD_SIZE, 0);
                                    else
                                        state <= RX_WORD;
                                end

                                CMD_CONFIG_END: begin
                                    if (
                                        !model_loading ||
                                        image_loaded != image_size
                                    ) begin
                                        reply(STATUS_BAD_STATE, image_loaded);
                                    end else begin
                                        word_index <= 0;
                                        state <= HDR_READ;
                                    end
                                end

                                CMD_INPUT_END: begin
                                    if (
                                        !input_loading ||
                                        input_loaded !=
                                        ({16'd0, model_inputs} << 2)
                                    ) begin
                                        reply(STATUS_BAD_STATE, input_loaded);
                                    end else begin
                                        input_index <= 0;
                                        state <= INPUT_CHECK_READ;
                                    end
                                end

                                CMD_RUN: begin
                                    if (
                                        !model_valid ||
                                        !input_valid ||
                                        model_loading ||
                                        input_loading
                                    ) begin
                                        reply(STATUS_BAD_STATE, 0);
                                    end else begin
                                        input_valid <= 1'b0;
                                        results_valid <= 1'b0;
                                        state <= RUN_BEGIN;
                                    end
                                end

                                CMD_READ_WORD: begin
                                    if (!results_valid) begin
                                        reply(STATUS_BAD_STATE, 0);

                                    end else if (
                                        io_to_seq_data_i.offset[1:0] != 0 ||
                                        io_to_seq_data_i.offset >=
                                            32'(SHARED_BYTES)
                                    ) begin
                                        reply(STATUS_BAD_OFFSET, 0);

                                    end else begin
                                        read_memory(
                                            io_to_seq_data_i.offset,
                                            READ_REPLY
                                        );
                                    end
                                end

                                default:
                                    reply(STATUS_FORMAT_ERROR, 0);

                            endcase
                        end
                    end
                end

                REPLY: begin
                    if (seq_to_io_valid_o && seq_to_io_ready_i)
                        state <= IDLE;
                end

                RX_WORD: begin
                    if (io_to_seq_valid_i && io_to_seq_ready_o) begin
                        rx_word <= io_to_seq_data_i.word;

                        if (rx_error != STATUS_OK) begin
                            state <= RX_NEXT;

                        end else if (
                            io_to_seq_data_i.is_header ||
                            io_to_seq_data_i.keep != 4'hF ||
                            io_to_seq_data_i.last != (rx_remaining <= 4)
                        ) begin
                            rx_error <= STATUS_BAD_STREAM;
                            state <= RX_NEXT;

                        end else begin
                            write_memory(
                                (rx_is_input ? input_addr[0] : 0) +
                                    rx_offset + rx_received,
                                io_to_seq_data_i.word,
                                RX_AFTER_WRITE
                            );
                        end
                    end
                end

                RX_AFTER_WRITE: begin
                    state <= RX_NEXT;
                end

                RX_NEXT: begin
                    if (rx_remaining <= 4) begin
                        if (rx_error == STATUS_OK) begin
                            if (rx_is_input)
                                input_loaded <= input_loaded + rx_length;
                            else
                                image_loaded <= image_loaded + rx_length;

                            reply(STATUS_OK, rx_offset + rx_length);

                        end else begin
                            reply(
                                rx_error,
                                rx_is_input ? input_loaded : image_loaded
                            );
                        end
                    end else begin
                        rx_remaining <= rx_remaining - 4;
                        rx_received <= rx_received + 4;
                        state <= RX_WORD;
                    end
                end

                MEM_REQ: begin
                    if (seq_to_mem_valid_o && seq_to_mem_ready_i)
                        state <= MEM_WAIT;
                end

                MEM_WAIT: begin
                    if (mem_to_seq_valid_i && mem_to_seq_ready_o) begin
                        if (mem_to_seq_data_i.error) begin
                            if (mem_return == RX_AFTER_WRITE) begin
                                // Сначала дочитываем остаток пакета.
                                rx_error <= STATUS_MEMORY_ERROR;
                                state <= RX_NEXT;
                            end else begin
                                fail(STATUS_MEMORY_ERROR, model_loading);
                            end
                        end else begin
                            memory_value <= mem_to_seq_data_i.value;
                            state <= mem_return;
                        end
                    end
                end

                PE_SEND: begin
                    if (|pe_fault) begin
                        fail(STATUS_PE_ERROR, model_loading);
                    end else begin
                        pending_pe <=
                            pending_pe & ~pe_to_seq_command_ready_i;

                        if (
                            (pending_pe & ~pe_to_seq_command_ready_i) == 0
                        )
                            state <= pe_return;
                    end
                end

                HDR_READ: begin
                    read_memory(
                        32'(word_index) << 2,
                        HDR_GOT
                    );
                end

                HDR_GOT: begin
                    header[word_index] <= memory_value;

                    if (word_index == 7) begin
                        state <= HDR_CHECK;
                    end else begin
                        word_index <= word_index + 1;
                        state <= HDR_READ;
                    end
                end

                HDR_CHECK: begin
                    if (
                        header[0] != 32'h32504E4E ||
                        header[1][15:0] != 1 ||

                        header[1][31:16] == 0 ||
                        int'(header[1][31:16]) > MAX_LAYERS ||

                        header[2][15:0] == 0 ||
                        int'(header[2][15:0]) > MAX_NEURONS ||

                        header[2][31:16] == 0 ||
                        int'(header[2][31:16]) > MAX_NEURONS ||

                        header[3] != 32'h00000101 ||

                        header[4] < 32 ||
                        header[4][1:0] != 0 ||

                        {32'd0, header[4]} +
                        ({48'd0, header[1][31:16]} << 5)
                            > {32'd0, image_size} ||

                        header[5] != image_size ||
                        header[7] != 0
                    ) begin
                        fail(STATUS_BAD_MODEL, 1'b1);
                    end else begin
                        layer_count <= header[1][31:16];
                        model_inputs <= header[2][15:0];
                        model_outputs <= header[2][31:16];

                        descriptor_offset <= header[4];
                        expected_crc <= header[6];

                        crc_reg <= 32'hFFFFFFFF;
                        word_index <= 0;
                        state <= CRC_READ;
                    end
                end

                CRC_READ: begin
                    read_memory(
                        32'(word_index) << 2,
                        CRC_GOT
                    );
                end

                CRC_GOT: begin
                    // Поле CRC самого образа считается нулём.
                    crc_reg <= crc_word(
                        crc_reg,
                        (word_index == 6) ? 32'd0 : memory_value
                    );

                    if (
                        ((32'(word_index) + 1) << 2) == image_size
                    ) begin
                        if (
                            (
                                crc_word(
                                    crc_reg,
                                    (word_index == 6)
                                        ? 32'd0 : memory_value
                                ) ^ 32'hFFFFFFFF
                            ) != expected_crc
                        ) begin
                            fail(STATUS_MODEL_CRC, 1'b1);
                        end else begin
                            layer_index <= 0;
                            word_index <= 0;
                            reserved_weights <= 0;
                            state <= DESC_READ;
                        end
                    end else begin
                        word_index <= word_index + 1;
                        state <= CRC_READ;
                    end
                end

                DESC_READ: begin
                    read_memory(
                        descriptor_offset +
                        (32'(layer_index) << 5) +
                        (32'(word_index) << 2),
                        DESC_GOT
                    );
                end

                DESC_GOT: begin
                    descriptor[word_index] <= memory_value;

                    if (word_index == 7) begin
                        state <= DESC_CHECK;
                    end else begin
                        word_index <= word_index + 1;
                        state <= DESC_READ;
                    end
                end

                DESC_CHECK: begin
                    if (!desc_ok) begin
                        fail(STATUS_BAD_MODEL, 1'b1);
                    end else begin
                        layer_n[layer_index] <= dn;
                        layer_m[layer_index] <= dm;

                        weight_offset[layer_index] <= descriptor[2];
                        bias_offset[layer_index] <= descriptor[3];

                        input_addr[layer_index] <= descriptor[4];
                        output_addr[layer_index] <= descriptor[5];

                        local_w_base[layer_index] <=
                            reserved_weights << 2;

                        weight_count[layer_index] <= dtotal;

                        quota[layer_index] <=
                            dtotal / 32'(N_PE);

                        remainder[layer_index] <=
                            dtotal % 32'(N_PE);

                        i_step[layer_index] <=
                            16'(N_PE / int'(dm));

                        j_step[layer_index] <=
                            16'(N_PE % int'(dm));

                        reserved_weights <=
                            reserved_weights + dreserve;

                        if (layer_index == int'(layer_count)-1) begin
                            local_x_base <=
                                (reserved_weights + dreserve) << 2;

                            local_s_base <=
                                (
                                    reserved_weights +
                                    dreserve +
                                    32'(MAX_NEURONS)
                                ) << 2;

                            layer_index <= 0;
                            state <= WEIGHTS_BEGIN;
                        end else begin
                            layer_index <= layer_index + 1;
                            word_index <= 0;
                            state <= DESC_READ;
                        end
                    end
                end

                WEIGHTS_BEGIN: begin
                    word_index <= 0;
                    pe_index <= 0;
                    local_index <= 0;
                    state <= WEIGHT_READ;
                end

                WEIGHT_READ: begin
                    read_memory(
                        weight_offset[layer_index] +
                        (32'(word_index) << 2),
                        WEIGHT_GOT
                    );
                end

                WEIGHT_GOT: begin
                    if (memory_value[30:23] == 8'hFF) begin
                        fail(STATUS_BAD_MODEL, 1'b1);
                    end else begin
                        pe_command <= '0;
                        pe_command.op <= PE_WRITE;

                        pe_command.addr <=
                            local_w_base[layer_index] +
                            (32'(local_index) << 2);

                        pe_command.value <= memory_value;

                        send_pe(
                            {{(N_PE-1){1'b0}}, 1'b1} << pe_index,
                            WEIGHT_NEXT
                        );
                    end
                end

                WEIGHT_NEXT: begin
                    if (
                        word_index ==
                        int'(weight_count[layer_index])-1
                    ) begin
                        state <= BIAS_VALIDATE_BEGIN;
                    end else begin
                        word_index <= word_index + 1;

                        if (pe_index == N_PE-1) begin
                            pe_index <= 0;
                            local_index <= local_index + 1;
                        end else begin
                            pe_index <= pe_index + 1;
                        end

                        state <= WEIGHT_READ;
                    end
                end

                BIAS_VALIDATE_BEGIN: begin
                    neuron_index <= 0;
                    state <= BIAS_VALIDATE_READ;
                end

                BIAS_VALIDATE_READ: begin
                    read_memory(
                        bias_offset[layer_index] +
                        (32'(neuron_index) << 2),
                        BIAS_VALIDATE_GOT
                    );
                end

                BIAS_VALIDATE_GOT: begin
                    if (memory_value[30:23] == 8'hFF) begin
                        fail(STATUS_BAD_MODEL, 1'b1);

                    end else if (
                        neuron_index == int'(layer_m[layer_index])-1
                    ) begin
                        if (layer_index == int'(layer_count)-1) begin
                            state <= MODEL_READY;
                        end else begin
                            layer_index <= layer_index + 1;
                            state <= WEIGHTS_BEGIN;
                        end
                    end else begin
                        neuron_index <= neuron_index + 1;
                        state <= BIAS_VALIDATE_READ;
                    end
                end

                MODEL_READY: begin
                    if (|pe_fault) begin
                        fail(STATUS_PE_ERROR, 1'b1);
                    end else begin
                        model_valid <= 1'b1;
                        model_loading <= 1'b0;
                        reply(STATUS_OK, image_size);
                    end
                end

                INPUT_CHECK_READ: begin
                    read_memory(
                        input_addr[0] + (32'(input_index) << 2),
                        INPUT_CHECK_GOT
                    );
                end

                INPUT_CHECK_GOT: begin
                    if (memory_value[30:23] == 8'hFF) begin
                        input_loading <= 1'b0;
                        reply(STATUS_BAD_STREAM, 0);

                    end else if (
                        input_index == int'(model_inputs)-1
                    ) begin
                        input_valid <= 1'b1;
                        input_loading <= 1'b0;
                        reply(STATUS_OK, input_loaded);

                    end else begin
                        input_index <= input_index + 1;
                        state <= INPUT_CHECK_READ;
                    end
                end

                READ_REPLY: begin
                    reply(STATUS_OK, memory_value);
                end

                RUN_BEGIN: begin
                    layer_index <= 0;
                    best_value <= 0;
                    best_class <= 0;

                    write_memory(
                        RESULT_RECORD + 12,
                        32'd1,
                        LAYER_BEGIN
                    );
                end

                LAYER_BEGIN: begin
                    pe_index <= 0;
                    first_i <= 0;
                    first_j <= 0;
                    state <= CFG_BUILD;
                end

                CFG_BUILD: begin
                    pe_command <= '0;
                    pe_command.op <= PE_CONFIG;

                    pe_command.cfg.n_in <= layer_n[layer_index];
                    pe_command.cfg.n_out <= layer_m[layer_index];

                    pe_command.cfg.w_base <=
                        local_w_base[layer_index];

                    pe_command.cfg.x_base <= local_x_base;
                    pe_command.cfg.s_base <= local_s_base;

                    pe_command.cfg.mac_count <=
                        quota[layer_index] +
                        (
                            32'(pe_index) < remainder[layer_index]
                                ? 32'd1 : 32'd0
                        );

                    pe_command.cfg.i_first <= first_i;
                    pe_command.cfg.j_first <= first_j;

                    pe_command.cfg.i_step <= i_step[layer_index];
                    pe_command.cfg.j_step <= j_step[layer_index];

                    send_pe(
                        {{(N_PE-1){1'b0}}, 1'b1} << pe_index,
                        CFG_NEXT
                    );
                end

                CFG_NEXT: begin
                    if (pe_index == N_PE-1) begin
                        input_index <= 0;
                        state <= X_READ;
                    end else begin
                        pe_index <= pe_index + 1;

                        if (
                            first_j == layer_m[layer_index] - 16'd1
                        ) begin
                            first_j <= 0;
                            first_i <= first_i + 16'd1;
                        end else begin
                            first_j <= first_j + 16'd1;
                        end

                        state <= CFG_BUILD;
                    end
                end

                X_READ: begin
                    read_memory(
                        input_addr[layer_index] +
                        (32'(input_index) << 2),
                        X_GOT
                    );
                end

                X_GOT: begin
                    if (memory_value[30:23] == 8'hFF) begin
                        fail(STATUS_COMPUTE_ERROR, 1'b0);
                    end else begin
                        pe_command <= '0;
                        pe_command.op <= PE_WRITE;

                        pe_command.addr <=
                            local_x_base +
                            (32'(input_index) << 2);

                        pe_command.value <= memory_value;

                        send_pe({N_PE{1'b1}}, X_NEXT);
                    end
                end

                X_NEXT: begin
                    if (
                        input_index == int'(layer_n[layer_index])-1
                    ) begin
                        state <= START_PES;
                    end else begin
                        input_index <= input_index + 1;
                        state <= X_READ;
                    end
                end

                START_PES: begin
                    pe_command <= '0;
                    pe_command.op <= PE_START;

                    send_pe({N_PE{1'b1}}, WAIT_PES);
                end

                WAIT_PES: begin
                    if (|pe_fault) begin
                        fail(STATUS_PE_ERROR, 1'b0);
                    end else if (&pe_to_seq_layer_done_i) begin
                        neuron_index <= 0;
                        state <= BIAS_READ;
                    end
                end

                BIAS_READ: begin
                    read_memory(
                        bias_offset[layer_index] +
                        (32'(neuron_index) << 2),
                        BIAS_GOT
                    );
                end

                BIAS_GOT: begin
                    bias_reg <= memory_value;
                    state <= RS_START;
                end

                RS_START: begin
                    if (|pe_fault) begin
                        fail(STATUS_PE_ERROR, 1'b0);

                    end else if (
                        seq_to_rs_start_o &&
                        rs_to_seq_start_ready_i
                    ) begin
                        state <= RS_WAIT;
                    end
                end

                RS_WAIT: begin
                    if (|pe_fault) begin
                        fail(STATUS_PE_ERROR, 1'b0);

                    end else if (
                        rs_to_seq_result_valid_i &&
                        seq_to_rs_result_ready_o
                    ) begin
                        if (
                            rs_to_seq_result_status_i != RS_OK ||
                            rs_to_seq_result_data_i[30:23] == 8'hFF
                        ) begin
                            fail(STATUS_COMPUTE_ERROR, 1'b0);
                        end else begin
                            // Sigmoid выдаёт неотрицательные значения.
                            // При равенстве сохраняется меньший индекс.
                            if (
                                layer_index == int'(layer_count)-1 &&
                                (
                                    neuron_index == 0 ||
                                    rs_to_seq_result_data_i > best_value
                                )
                            ) begin
                                best_value <= rs_to_seq_result_data_i;
                                best_class <= 32'(neuron_index);
                            end

                            write_memory(
                                output_addr[layer_index] +
                                (32'(neuron_index) << 2),
                                rs_to_seq_result_data_i,
                                RESULT_STORED
                            );
                        end
                    end
                end

                RESULT_STORED: begin
                    // Это состояние достигается только после
                    // успешного ответа памяти на запись результата.
                    if (
                        neuron_index == int'(layer_m[layer_index])-1
                    ) begin
                        if (layer_index == int'(layer_count)-1) begin
                            record_index <= 0;
                            state <= WRITE_RECORD;
                        end else begin
                            layer_index <= layer_index + 1;
                            state <= LAYER_BEGIN;
                        end
                    end else begin
                        state <= NEXT_PES;
                    end
                end

                NEXT_PES: begin
                    pe_command <= '0;
                    pe_command.op <= PE_NEXT;

                    send_pe({N_PE{1'b1}}, NEXT_NEURON);
                end

                NEXT_NEURON: begin
                    neuron_index <= neuron_index + 1;
                    state <= BIAS_READ;
                end

                WRITE_RECORD: begin
                    case (record_index)

                        0: write_memory(
                            RESULT_RECORD,
                            best_class,
                            RECORD_NEXT
                        );

                        1: write_memory(
                            RESULT_RECORD + 4,
                            {16'd0, model_outputs},
                            RECORD_NEXT
                        );

                        2: write_memory(
                            RESULT_RECORD + 8,
                            output_addr[layer_index],
                            RECORD_NEXT
                        );

                        3: write_memory(
                            RESULT_RECORD + 12,
                            32'd0,
                            RECORD_NEXT
                        );

                        default:
                            fail(STATUS_COMPUTE_ERROR, 1'b0);

                    endcase
                end

                RECORD_NEXT: begin
                    if (record_index == 3) begin
                        results_valid <= 1'b1;

                        reply(
                            STATUS_OK,
                            output_addr[layer_index]
                        );
                    end else begin
                        record_index <= record_index + 1;
                        state <= WRITE_RECORD;
                    end
                end

                default:
                    fail(STATUS_BAD_STATE, 1'b1);

            endcase
        end
    end

endmodule