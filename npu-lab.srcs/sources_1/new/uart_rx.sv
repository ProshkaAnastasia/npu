module uart_rx #(
    parameter int CLK_FREQ_HZ = 100_000_000,
    parameter int BAUD_RATE   = 115_200
) (
    input  logic       clk_i,
    input  logic       rst_n_i,

    input  logic       rx_i,

    output logic [7:0] byte_o,
    output logic       valid_o,
    output logic       error_o
);

    localparam int DIV  = CLK_FREQ_HZ / BAUD_RATE;
    localparam int CW   = (DIV > 1) ? $clog2(DIV) : 1;
    localparam int HALF = DIV / 2;

    typedef enum logic [2:0] {
        IDLE,
        START,
        DATA,
        STOP,
        WAIT_HIGH
    } state_t;

    state_t state;

    // Двухступенчатая синхронизация линии RX.
    (* ASYNC_REG = "TRUE" *) logic rx_meta;
    (* ASYNC_REG = "TRUE" *) logic rx_sync;

    logic [CW-1:0] count;
    logic [2:0]    bit_index;
    logic [7:0]    shift;

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx_i;
            rx_sync <= rx_meta;
        end
    end

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            state     <= IDLE;
            count     <= '0;
            bit_index <= '0;
            shift     <= '0;

            byte_o  <= '0;
            valid_o <= 1'b0;
            error_o <= 1'b0;

        end else begin
            // valid и error - импульсы на один такт.
            valid_o <= 1'b0;
            error_o <= 1'b0;

            case (state)

                IDLE: begin
                    count <= '0;

                    if (!rx_sync)
                        state <= START;
                end

                START: begin
                    // Проверка стартового бита в середине.
                    if (count == $unsigned(CW'(HALF - 1))) begin
                        count <= '0;

                        if (!rx_sync) begin
                            bit_index <= '0;
                            state <= DATA;
                        end else begin
                            // Ложный старт.
                            state <= IDLE;
                        end

                    end else begin
                        count <= count + 1'b1;
                    end
                end

                DATA: begin
                    if (count == $unsigned(CW'(DIV - 1))) begin
                        count <= '0;

                        shift[bit_index] <= rx_sync;

                        if (bit_index == 3'd7)
                            state <= STOP;
                        else
                            bit_index <= bit_index + 1'b1;

                    end else begin
                        count <= count + 1'b1;
                    end
                end

                STOP: begin
                    if (count == $unsigned(CW'(DIV - 1))) begin
                        count <= '0;

                        if (rx_sync) begin
                            byte_o  <= shift;
                            valid_o <= 1'b1;
                            state   <= IDLE;
                        end else begin
                            error_o <= 1'b1;
                            state   <= WAIT_HIGH;
                        end

                    end else begin
                        count <= count + 1'b1;
                    end
                end

                WAIT_HIGH: begin
                    // После ошибки стопового бита
                    // ждём возврата линии в единицу.
                    if (rx_sync)
                        state <= IDLE;
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule