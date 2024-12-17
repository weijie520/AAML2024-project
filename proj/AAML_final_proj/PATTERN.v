`include "global_buffer_bram.v"
`include "TPU.v"


module PATTERN(
    clk,
    reset,

    func,
    state,

    InputOffset,
    K,
    M,
    N,

    A_index,
    A_data_in,

    B_index,
    B_data_in,

    C_index,
    C_data_out
);

input               clk;
input               reset;
input      [6:0]    func;
output reg [1:0]    state;

input      [8:0]    InputOffset;
input      [7:0]    K;
input      [7:0]    M;
input      [7:0]    N;

wire                A_wr_en;
input      [11:0]   A_index;
input      [31:0]   A_data_in;

wire                B_wr_en;
input      [11:0]   B_index;
input      [31:0]   B_data_in;

wire                C_wr_en;
input      [11:0]   C_index;
wire       [127:0]  C_data_in;
output     [127:0]  C_data_out;

reg                 in_valid;
reg        [7:0]    k;
reg        [7:0]    m;
reg        [7:0]    n;

assign A_wr_en = (func == 1);
assign B_wr_en = (func == 2);
wire  [11:0] a_index, b_index, c_index;
wire  [31:0] A_data_out, B_data_out;
wire  [11:0] tpu_a_index, tpu_b_index, tpu_c_index;
reg   [8:0] input_offset;

wire   tpu_busy;
assign a_index = (func == 3) ? tpu_a_index : A_index;
assign b_index = (func == 3) ? tpu_b_index : B_index;
assign c_index = (func == 3) ? tpu_c_index : C_index;


global_buffer_bram #(
  .ADDR_BITS(12), // ADDR_BITS 12 -> generates 2^12 entries
  .DATA_BITS(32)  // DATA_BITS 32 -> 32 bits for each entries
)
gbuff_A(
  .clk(clk),
  .rst_n(1'b1),
  .ram_en(1'd1),
  .wr_en(A_wr_en),
  .index(a_index),
  .data_in(A_data_in),
  .data_out(A_data_out)
);

global_buffer_bram #(
  .ADDR_BITS(12), // ADDR_BITS 12 -> generates 2^12 entries
  .DATA_BITS(32)  // DATA_BITS 32 -> 32 bits for each entries
)
gbuff_B(
  .clk(clk),
  .rst_n(1'b1),
  .ram_en(1'd1),
  .wr_en(B_wr_en),
  .index(b_index),
  .data_in(B_data_in),
  .data_out(B_data_out)
);

global_buffer_bram #(
  .ADDR_BITS(12), // ADDR_BITS 12 -> generates 2^12 entries
  .DATA_BITS(128)  // DATA_BITS 32 -> 32 bits for each entries
)
gbuff_C(
  .clk(clk),
  .rst_n(1'b1),
  .ram_en(1'b1),
  .wr_en(C_wr_en),
  .index(c_index),
  .data_in(C_data_in),
  .data_out(C_data_out)
);

// TPU
TPU My_TPU(
  .clk(clk),
  .rst_n(~reset),
  .in_valid(in_valid),
  .InputOffset(input_offset),
  .K(k),
  .M(m),
  .N(n),
  .busy(tpu_busy),
  .A_wr_en(0),
  .A_index(tpu_a_index),
  .A_data_in(1'd1),
  .A_data_out(A_data_out),
  .B_wr_en(0),
  .B_index(tpu_b_index),
  .B_data_in(1'd1),
  .B_data_out(B_data_out),
  .C_wr_en(C_wr_en),
  .C_index(tpu_c_index),
  .C_data_in(C_data_in),
  .C_data_out(1'd0)
);


always @ (negedge clk) begin
  if(reset) begin
    in_valid = 1'b0;
    k = 0;
    m = 0;
    n = 0;
    state = 2'd0;
  end else begin
    if(func == 1 || func == 2) begin
      state = 2'd0;
      in_valid = 1'b0;
    end else if(func == 3) begin
      case(state)
        0: begin
          in_valid = 1'b1;
          k = K;
          m = M;
          n = N;
          state = 2'd1;
        end
        1: begin
          if(tpu_busy) begin
            in_valid = 1'b0;
            state = 2'd2;
          end
        end
        2: begin
          if(tpu_busy == 0) begin
            state = 2'd3;
          end
        end
        default: begin
          in_valid = 1'b0;
          k = 0;
          m = 0;
          n = 0;
        end
      endcase
    end else if(func == 5) begin
      input_offset <= InputOffset;
    end
  end
end

endmodule
