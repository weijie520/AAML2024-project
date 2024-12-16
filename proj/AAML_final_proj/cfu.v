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
            case(state)
              0: begin
                rsp_payload_outputs_0 <= SaturatingRoundingDoublingHighMul(cmd_payload_inputs_0, cmd_payload_inputs_1);
                rsp_valid <= 1'b1;
                cmd_ready <= 1'b0;
                state <= 1;
              end
              1: begin
                rsp_payload_outputs_0 <= cmd_payload_inputs_1 + RoundingDivideByPOT(rsp_payload_outputs_0, cmd_payload_inputs_0);
                rsp_valid <= 1'b1;
                cmd_ready <= 1'b0;
                state <= 0;
              end
            endcase
          end
        endcase
      end
    end
  end

  // Multiply two fixpoint raw values, should be care for the kIntergerBits of result
  function signed [31:0] SaturatingRoundingDoublingHighMul;
    input signed [31:0] a, b;
    reg signed [63:0] result;
    // reg signed [31:0] nudge;
    begin
      result = a * b;

      // Rounding logic based on sign of the result
      // if (result >= 0)
      //     nudge = (1 << 30);  // Round up for positive values
      // else
      //     nudge = (1 - (1 << 30));  // Round down for negative values

      // Apply the rounding and shift the result to 32 bits
      // result = (result + nudge) >>> 31;  // Rounding3
      result = result >>> 31;  // Rounding
      SaturatingRoundingDoublingHighMul = result[31:0];  // Return lower 32 bits
    end
  endfunction

  function signed [31:0] RoundingDivideByPOT;
    input signed [31:0] x, exponent;
    reg signed [31:0] mask, remainder, result;
    begin
      result = x >>> exponent;
      RoundingDivideByPOT = result;
    end
  endfunction

endmodule