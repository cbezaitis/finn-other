// Copyright (c) 2026 Norwegian University of Science and Technology (NTNU)
// SPDX-License-Identifier: BSD-3-Clause

module delta_thresholding #(
    parameter int unsigned N = 1,
    parameter int unsigned O_BITS = 1,
    parameter int unsigned WI = 1,
    parameter int unsigned WT = 1,
    parameter int unsigned BW = 1,
    parameter int unsigned SW = 1,
    parameter int unsigned EW = 1,
    parameter int unsigned CW = 1,
    parameter int unsigned C = 1,
    parameter int unsigned PE = 1,
    parameter int unsigned NUM_STEPS = 1,
    parameter bit SIGNED = 1,
    parameter bit BASE_SIGNED = 1,
    parameter bit STEP_SIGNED = 1,
    parameter int BIAS = 0,
    // Same style as thresholding.sv THRESHOLDS_PATH (untyped string parameter).
    parameter BASE_PATH = "",
    parameter STEP_PATH = "",
    parameter ERROR_PATH = "",
    parameter COUNT_PATH = ""
) (
    input logic clk,
    input logic rst,
    output logic irdy,
    input logic ivld,
    input logic [PE-1:0][WI-1:0] idat,
    input logic ordy,
    output logic ovld,
    output logic [PE-1:0][O_BITS-1:0] odat
);
    localparam int unsigned CF = C / PE;
    localparam int unsigned FOLD_BITS = CF <= 1 ? 1 : $clog2(CF);
    localparam int unsigned STEP_BITS = NUM_STEPS <= 1 ? 1 : $clog2(NUM_STEPS);
    localparam int unsigned COMP_W = (WT > WI ? WT : WI) + 1;
    localparam int unsigned PARAM_COUNT = PE * CF;

    // PE-major flat memories are directly initialized by $readmemh. Vivado
    // synthesis does not preserve an initial copy from generated local arrays
    // into a shared multidimensional memory, leaving the latter undriven.
    (* rom_style = "distributed" *) logic [BW-1:0] base_mem [PARAM_COUNT];
    (* rom_style = "distributed" *) logic [SW-1:0] step_mem [PARAM_COUNT];
    (* rom_style = "distributed" *) logic [0:0] error_mem [PARAM_COUNT * NUM_STEPS];
    (* rom_style = "distributed" *) logic [CW-1:0] count_mem [PARAM_COUNT];
    logic [PE-1:0][WI-1:0] input_reg;
    logic [PE-1:0][O_BITS-1:0] count_reg;
    logic [PE-1:0][EW-1:0] residual_reg;
    logic [FOLD_BITS-1:0] fold_reg;
    logic [STEP_BITS-1:0] step_index;
    logic busy;
    logic output_valid;
    logic [PE-1:0][O_BITS-1:0] output_reg;

    initial begin
        if (CF * PE != C) begin
            $error("Delta thresholding requires C to be divisible by PE.");
            $finish;
        end
        if (BASE_PATH != "")
            $readmemh(BASE_PATH, base_mem);
        if (STEP_PATH != "")
            $readmemh(STEP_PATH, step_mem);
        if (ERROR_PATH != "")
            $readmemh(ERROR_PATH, error_mem);
        if (COUNT_PATH != "")
            $readmemh(COUNT_PATH, count_mem);
    end

    assign irdy = !busy && (!output_valid || ordy);
    assign ovld = output_valid;
    assign odat = output_reg;

    always_ff @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0;
            output_valid <= 1'b0;
            fold_reg <= '0;
            step_index <= '0;
            input_reg <= '0;
            count_reg <= '0;
            residual_reg <= '0;
            output_reg <= '0;
        end else begin
            if (output_valid && ordy)
                output_valid <= 1'b0;

            if (!busy && irdy && ivld) begin
                input_reg <= idat;
                count_reg <= '0;
                residual_reg <= '0;
                step_index <= '0;
                busy <= 1'b1;
                // Keep fold_reg stable while busy: advancing here would make the
                // NUM_STEPS compare cycles use the *next* channel's parameters
                // (stock tests hid this when duplicated threshold rows made CF>1
                // folds share identical thresholds).
            end else if (busy) begin
                for (int pe = 0; pe < PE; pe++) begin
                    logic signed [COMP_W-1:0] base_value;
                    logic signed [COMP_W-1:0] step_value;
                    logic signed [COMP_W-1:0] threshold_value;
                    logic signed [COMP_W-1:0] input_value;
                    logic [EW-1:0] residual_value;
                    logic comparison;
                    logic [CW-1:0] count_value;
                    integer error_index;
                    integer parameter_index;
                    integer output_value;
                    parameter_index = pe * CF + int'(fold_reg);
                    if (BASE_SIGNED)
                        base_value = $signed(
                            {{(COMP_W-BW){base_mem[parameter_index][BW-1]}},
                             base_mem[parameter_index]}
                        );
                    else
                        base_value = $signed(
                            {{(COMP_W-BW){1'b0}}, base_mem[parameter_index]}
                        );
                    if (STEP_SIGNED)
                        step_value = $signed(
                            {{(COMP_W-SW){step_mem[parameter_index][SW-1]}},
                             step_mem[parameter_index]}
                        );
                    else
                        step_value = $signed(
                            {{(COMP_W-SW){1'b0}}, step_mem[parameter_index]}
                        );
                    error_index = parameter_index * NUM_STEPS + int'(step_index);
                    residual_value = residual_reg[pe] + error_mem[error_index][0];
                    count_value = count_mem[parameter_index];
                    threshold_value = base_value
                        + step_value * $signed({1'b0, step_index})
                        + residual_value;
                    if (SIGNED)
                        input_value = $signed(
                            {{(COMP_W-WI){input_reg[pe][WI-1]}}, input_reg[pe]}
                        );
                    else
                        input_value = $signed({{(COMP_W-WI){1'b0}}, input_reg[pe]});
                    comparison = (step_index < count_value) && (threshold_value <= input_value);
                    count_reg[pe] <= count_reg[pe] + comparison;
                    residual_reg[pe] <= residual_value;
                    output_value = int'(count_reg[pe]) + int'(comparison) + BIAS;
                    if (step_index == STEP_BITS'(NUM_STEPS - 1))
                        output_reg[pe] <= output_value[O_BITS-1:0];
                end

                if (step_index == STEP_BITS'(NUM_STEPS - 1)) begin
                    busy <= 1'b0;
                    output_valid <= 1'b1;
                    if (CF > 1) begin
                        if (fold_reg == FOLD_BITS'(CF - 1))
                            fold_reg <= '0;
                        else
                            fold_reg <= fold_reg + 1'b1;
                    end
                end else begin
                    step_index <= step_index + 1'b1;
                end
            end
        end
    end
endmodule
