module PE(
  clk,
  rst_n,

  busy,
  clear,

  InputOffset,
  a_in,
  a_out,

  b_in,
  b_out,

  acc
);

input clk;
input rst_n;

input busy;
input clear;


input [8:0] InputOffset;
input [7:0] a_in;
output reg [7:0] a_out;

input [7:0] b_in;
output reg [7:0] b_out;
output reg [31:0] acc;
reg signed [16:0] tmp;

always @(posedge clk or negedge rst_n) begin
  if(~rst_n) begin
    a_out <= 8'b0;
    b_out <= 8'b0;
    acc <= 32'b0;
    tmp <= 32'b0;
  end else begin
    if(clear) begin
      acc <= 32'b0;
      a_out <= 8'b0;
      b_out <= 8'b0;
      tmp <= 32'b0;
    end else if(busy) begin
      tmp <= ($signed(a_in) + $signed(InputOffset)) * $signed(b_in);
      acc <= $signed(acc) + $signed(tmp);
      a_out <= a_in;
      b_out <= b_in;
    end
  end
end

endmodule