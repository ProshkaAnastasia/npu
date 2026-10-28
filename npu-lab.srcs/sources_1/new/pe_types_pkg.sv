`timescale 1ns/1ps

package pe_types_pkg;

    // Конфигурация одного PE для текущего слоя.
    // Все адреса - локальные байтовые адреса.
    typedef struct packed {
        logic [15:0] n_in;
        logic [15:0] n_out;

        logic [31:0] w_base;
        logic [31:0] x_base;
        logic [31:0] s_base;

        logic [31:0] mac_count;

        logic [15:0] i_first;
        logic [15:0] j_first;

        logic [15:0] i_step;
        logic [15:0] j_step;
    } pe_cfg_t;

    // Команды секвенсора для PE.
    typedef enum logic [1:0] {
        PE_CONFIG = 2'd0,
        PE_WRITE  = 2'd1,
        PE_START  = 2'd2,
        PE_NEXT   = 2'd3
    } pe_op_t;

    // Секвенсор -> PE.
    //
    // CONFIG использует cfg.
    // WRITE использует addr и value.
    // START и NEXT дополнительных данных не требуют.
    typedef struct packed {
        pe_op_t op;

        pe_cfg_t cfg;

        logic [31:0] addr;
        logic [31:0] value;
    } seq_to_pe_t;

    // Статусы PE.
    typedef enum logic [2:0] {
        PE_OK       = 3'd0,
        PE_BAD_CFG  = 3'd1,
        PE_BAD_CMD  = 3'd2,
        PE_BAD_ADDR = 3'd3,
        PE_BAD_FP   = 3'd4
    } pe_status_t;

    // AGU -> PE.
    // Адреса операндов текущей MAC.
    typedef struct packed {
        logic [31:0] w_addr;
        logic [31:0] x_addr;
        logic [31:0] s_addr;

        logic last;
    } agu_to_pe_t;

endpackage