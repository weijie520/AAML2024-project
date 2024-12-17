`include "PATTERN.v"
module Cfu (
    input               cmd_valid,
    output reg          cmd_ready,
    input      [9:0]    cmd_payload_function_id,
    input      [31:0]   cmd_payload_inputs_0,
    input      [31:0]   cmd_payload_inputs_1,
    output reg          rsp_valid,
    input               rsp_ready,
    output reg [31:0]   rsp_payload_outputs_0,
    input               reset,
    input               clk
);

reg [6:0] func;
reg [4:0] state;
reg [31:0] index = 0;
wire  [1:0]     tpu_state;
reg   [7:0]     K;
reg   [7:0]     M;
reg   [7:0]     N;
wire  [11:0]    A_index;
reg   [31:0]    A_data_in;
wire  [11:0]    B_index;
reg   [31:0]    B_data_in;
wire  [11:0]    C_index;
wire  [127:0]   C_data_out;

reg [11:0] a_index, b_index, c_index;
reg [8:0] input_offset;

assign A_index = a_index;
assign B_index = b_index;
assign C_index = c_index;

PATTERN pattern(
  .clk(clk),
  .reset(reset),
  .func(func),
  .state(tpu_state),
  .InputOffset(input_offset),
  .K(K),
  .M(M),
  .N(N),
  .A_index(A_index),
  .A_data_in(A_data_in),
  .B_index(B_index),
  .B_data_in(B_data_in),
  .C_index(C_index),
  .C_data_out(C_data_out)
);

always @ (posedge clk) begin
  if(reset) begin
    rsp_payload_outputs_0 <= 32'd0;
    rsp_valid <= 1'b0;
    cmd_ready <= 1'b1;
    func <= 0;
    state <= 0;
  end else if(rsp_valid) begin
    rsp_valid <= ~rsp_ready;
    cmd_ready <= rsp_ready;
  end else if(cmd_valid) begin
    if(cmd_ready) begin
      func = cmd_payload_function_id[9:3];
      case(func)
        0: begin
          rsp_payload_outputs_0 <= SaturatingRoundingDoublingHighMul(cmd_payload_inputs_0, cmd_payload_inputs_1);
          rsp_valid <= 1'b1;
          cmd_ready <= 1'b0;
        end
        1: begin
          A_data_in = cmd_payload_inputs_0;
          a_index <= cmd_payload_inputs_1[11:0];
          rsp_valid <= 1'b1;
          cmd_ready <= 1'b0;
        end
        2: begin
          B_data_in = cmd_payload_inputs_0;
          b_index <= cmd_payload_inputs_1[11:0];
          rsp_valid <= 1'b1;
          cmd_ready <= 1'b0;
        end
        3: begin
          K <= cmd_payload_inputs_0[7:0];
          M <= cmd_payload_inputs_0[15:8];
          N <= cmd_payload_inputs_1[7:0];
          cmd_ready <= 1'b0;
          rsp_valid <= 1'b0;
          state <= 1;
        end
        4: begin
          c_index <= cmd_payload_inputs_0[11:0];
          index <= cmd_payload_inputs_1;
          cmd_ready <= 1'b0;
          rsp_valid <= 1'b0;
          state <= 1;
        end
        5: begin
          input_offset <= cmd_payload_inputs_0;
          cmd_ready <= 1'b0;
          rsp_valid <= 1'b1;
        end
        6: begin
          rsp_payload_outputs_0 <= RoundingDivideByPOT(rsp_payload_outputs_0, cmd_payload_inputs_1);
          rsp_valid <= 1'b1;
          cmd_ready <= 1'b0;
        end
        default: begin // for conv, fully_connected, pooling, softmax
          // rsp_payload_outputs_0 <= 32'd0;
          rsp_valid <= 1'b1;
          cmd_ready <= 1'b0;
        end
      endcase
    end
  end else if(func == 3 && state == 1) begin
    if(tpu_state == 3) begin
      rsp_payload_outputs_0 <= tpu_state;
      rsp_valid <= 1'b1;
      state <= 0;
    end
  end else if(func == 4) begin
    if(state == 1) begin
      case(index)
        0: begin
          rsp_payload_outputs_0 <= C_data_out[127:96];
        end
        1: begin
          rsp_payload_outputs_0 <= C_data_out[95:64];
        end
        2: begin
          rsp_payload_outputs_0 <= C_data_out[63:32];
        end
        3: begin
          rsp_payload_outputs_0 <= C_data_out[31:0];
        end
      endcase
      rsp_valid <= 1'b1;
      state <= 0;
    end
  end
end

// Multiply two fixpoint raw values, should be care for the kIntergerBits of result
function signed [31:0] SaturatingRoundingDoublingHighMul;
  input signed [31:0] a, b;
  reg signed [63:0] result;
  reg signed [31:0] nudge;
  reg overflow;
  begin
    overflow = (a == 32'h00000000) && (b == 32'h00000000);
    result = a * b;
    // Rounding logic based on sign of the result
    if (result >= 0)
        nudge = (1 << 30);  // Round up for positive values
    else
        nudge = (1 - (1 << 30));  // Round down for negative values

    // Apply the rounding and shift the result to 32 bits
    result = (result + nudge) >>> 31;  // Rounding
    SaturatingRoundingDoublingHighMul = overflow ? 32'h7fffffff : result;  // Return lower 32 bits
  end
endfunction

function signed [31:0] RoundingDivideByPOT;
  input signed [31:0] x, exponent;
  reg signed [31:0] mask, remainder, threshold, result;
  begin
    mask = (1 << exponent) - 1;
    remainder = x & mask;
    threshold = (mask >>> 1) + x[31]; // if(x < 0) threshold = mask >> 1 + 1
    RoundingDivideByPOT = $signed(x >>> exponent) + ($signed(remainder) > $signed(threshold));
  end
endfunction

endmodule