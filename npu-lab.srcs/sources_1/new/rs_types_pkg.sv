`timescale 1ns/1ps

package rs_types_pkg;

    typedef enum logic [1:0] {
        RS_OK          = 2'd0,
        RS_NONFINITE   = 2'd2,
        RS_ARITH_ERROR = 2'd3
    } rs_status_t;

endpackage