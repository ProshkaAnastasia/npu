module uart_tx #(
    parameter int CLK_FREQ_HZ = 100_000_000,
    parameter int BAUD_RATE   = 115_200
) (
    input  logic       clk_i,
    input  logic       rst_n_i,

    input  logic [7:0] byte_i,
    input  logic       valid_i,
    output logic       ready_o,

    output logic       tx_o
);

    localparam int DIV = CLK_FREQ_HZ / BAUD_RATE;
    localparam int CW  = (DIV > 1) ? $clog2(DIV) : 1;

    logic          active;
    logic [9:0]    frame;
    logic [3:0]    bit_index;
    logic [CW-1:0] count;

    assign ready_o = !active;

    assign tx_o = active
        ? frame[bit_index]
        : 1'b1;

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            active    <= 1'b0;
            frame     <= '1;
            bit_index <= '0;
            count     <= '0;

        end else if (!active) begin
            if (valid_i && ready_o) begin
                // frame[0]   - стартовый бит.
                // frame[8:1] - данные, младший бит первым.
                // frame[9]   - стоповый бит.
                frame <= {1'b1, byte_i, 1'b0};

                active    <= 1'b1;
                bit_index <= '0;
                count     <= '0;
            end

        end else if (count == $unsigned(CW'(DIV - 1))) begin
            count <= '0;

            if (bit_index == 4'd9)
                active <= 1'b0;
            else
                bit_index <= bit_index + 1'b1;

        end else begin
            count <= count + 1'b1;
        end
    end

endmodule