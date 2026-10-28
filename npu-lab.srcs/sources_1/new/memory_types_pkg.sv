`timescale 1ns/1ps

package memory_types_pkg;

    // «апрос к универсальному модулю memory.
    // »спользуетс€ и локальной, и общей пам€тью.
    typedef struct packed {
        logic        write;
        logic [31:0] addr;
        logic [31:0] value;
    } mem_req_t;

    // ќтвет пам€ти.
    //
    // ѕри чтении value содержит прочитанное слово.
    // ѕри записи value игнорируетс€.
    // error означает неверный адрес или выравнивание.
    typedef struct packed {
        logic [31:0] value;
        logic        error;
    } mem_rsp_t;

endpackage