`timescale 1ns/1ps

module pe_agu (
    input logic clk_i,
    input logic rst_n_i,

    input pe_types_pkg::pe_cfg_t cfg_i,

    input logic load_i,
    input logic advance_i,

    output pe_types_pkg::agu_to_pe_t agu_to_pe_data_o,
    output logic agu_to_pe_valid_o
);

    logic [31:0] w_addr;
    logic [31:0] x_addr;
    logic [31:0] s_addr;

    logic [31:0] remaining;

    logic [31:0] x_delta;
    logic [31:0] x_wrap_delta;

    logic [31:0] s_delta;
    logic [31:0] s_wrap_delta;

    logic [31:0] wrap_at;

    assign agu_to_pe_data_o.w_addr = w_addr;
    assign agu_to_pe_data_o.x_addr = x_addr;
    assign agu_to_pe_data_o.s_addr = s_addr;
    assign agu_to_pe_data_o.last   = (remaining == 1);

    assign agu_to_pe_valid_o = (remaining != 0);

    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            w_addr       <= 0;
            x_addr       <= 0;
            s_addr       <= 0;
            remaining    <= 0;

            x_delta      <= 0;
            x_wrap_delta <= 0;

            s_delta      <= 0;
            s_wrap_delta <= 0;

            wrap_at      <= 0;

        end else if (load_i) begin
            // Начальные адреса.
            w_addr <= cfg_i.w_base;

            x_addr <= cfg_i.x_base
                + ({16'd0, cfg_i.i_first} << 2);

            s_addr <= cfg_i.s_base
                + ({16'd0, cfg_i.j_first} << 2);

            remaining <= cfg_i.mac_count;

            // Приращения адреса входа.
            x_delta <= ({16'd0, cfg_i.i_step} << 2);

            x_wrap_delta <=
                (({16'd0, cfg_i.i_step} + 32'd1) << 2);

            // Приращения адреса суммы.
            s_delta <= ({16'd0, cfg_i.j_step} << 2);

            // Отрицательное приращение хранится
            // в дополнительном коде.
            s_wrap_delta <= (
                {16'd0, cfg_i.j_step}
                - {16'd0, cfg_i.n_out}
            ) << 2;

            // Граница перехода на следующую строку.
            wrap_at <= cfg_i.s_base + (
                (
                    {16'd0, cfg_i.n_out}
                    - {16'd0, cfg_i.j_step}
                ) << 2
            );

        end else if (advance_i && remaining != 0) begin
            remaining <= remaining - 32'd1;

            if (remaining > 1) begin
                w_addr <= w_addr + 32'd4;

                if (s_addr >= wrap_at) begin
                    x_addr <= x_addr + x_wrap_delta;
                    s_addr <= s_addr + s_wrap_delta;
                end else begin
                    x_addr <= x_addr + x_delta;
                    s_addr <= s_addr + s_delta;
                end
            end
        end
    end

endmodule