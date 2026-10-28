`timescale 1ns/1ps

package rs_types_pkg;

    // Секвенсор -> reduce + sigmoid.
    // Bias - значение FP32, не адрес.
    typedef struct packed {
        logic [15:0] neuron_id;
        logic [31:0] bias;
    } seq_to_rs_t;

    // PE -> reduce.
    typedef struct packed {
        logic [15:0] neuron_id;
        logic [31:0] value;
    } pe_to_rs_t;

    // Статусы обработки нейрона.
    typedef enum logic [1:0] {
        RS_OK          = 2'd0,
        RS_BAD_NEURON  = 2'd1,
        RS_NONFINITE   = 2'd2,
        RS_ARITH_ERROR = 2'd3
    } rs_status_t;

    // Reduce + sigmoid -> секвенсор.
    // Также используется между внутренними reduce и sigmoid.
    typedef struct packed {
        logic [15:0] neuron_id;
        logic [31:0] value;
        rs_status_t  status;
    } rs_to_seq_t;

endpackage