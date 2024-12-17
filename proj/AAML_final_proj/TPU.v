`include "PE.v"
module TPU(
    clk,
    rst_n,

    in_valid,

    InputOffset,
    K,
    M,
    N,
    busy,

    A_wr_en,
    A_index,
    A_data_in,
    A_data_out,

    B_wr_en,
    B_index,
    B_data_in,
    B_data_out,

    C_wr_en,
    C_index,
    C_data_in,
    C_data_out
);


input clk;
input rst_n;
input            in_valid;

input [8:0]      InputOffset;
input [7:0]      K;
input [7:0]      M;
input [7:0]      N;
output  reg      busy;

output           A_wr_en;
output [11:0]     A_index;
output [31:0]    A_data_in;
input  [31:0]    A_data_out;

output           B_wr_en;
output [11:0]     B_index;
output [31:0]    B_data_in;
input  [31:0]    B_data_out;

output           C_wr_en;
output [11:0]     C_index;
output [127:0]   C_data_in;
input  [127:0]   C_data_out;

reg [31:0] A_data_out_reg, B_data_out_reg;
reg [1:0] c_cnt;
reg [11:0] a_index, b_index, c_index;
reg [8:0] input_offset;

reg stop; // use to control the busy signal

// control
reg [3:0] control;

//* Implement your design here
assign A_wr_en = 0;
assign B_wr_en = 0;
// assign C_wr_en = ((control >= 7) && (M_count < M_reg)) ? 1 : 0;

assign A_data_in = 32'd0;
assign B_data_in = 32'd0;
assign C_data_in = {pe_acc[c_cnt][0], pe_acc[c_cnt][1], pe_acc[c_cnt][2], pe_acc[c_cnt][3]};

assign A_index = a_index;
assign B_index = b_index;
assign C_index = c_index;

always @(posedge clk or negedge rst_n) begin
  if(~rst_n) begin
    busy <= 1'd0;
  end else begin
    case (busy)
      0: begin
        busy <= in_valid ? 1'd1 : 1'd0;
      end
      1: begin
        busy <= (stop) ? 1'd0: 1'd1;
      end
    endcase
  end
end

// shift register
reg signed [7:0] r1 [0:0]; // row 1 need delay 1 cycle
reg signed [7:0] r2 [0:1];
reg signed [7:0] r3 [0:2];
reg signed [7:0] c1 [0:0]; // column 1 need delay 1 cycle
reg signed [7:0] c2 [0:1];
reg signed [7:0] c3 [0:2];

always @(posedge clk or negedge rst_n) begin
  if(~rst_n) begin
    r1[0] <= 8'd0;
    r2[0] <= 8'd0;
    r2[1] <= 8'd0;
    r3[0] <= 8'd0;
    r3[1] <= 8'd0;
    r3[2] <= 8'd0;
    c1[0] <= 8'd0;
    c2[0] <= 8'd0;
    c2[1] <= 8'd0;
    c3[0] <= 8'd0;
    c3[1] <= 8'd0;
    c3[2] <= 8'd0;
  end else begin
    if(busy) begin
      r3[2] <= A_data_out_reg[7:0];
      {r2[1], r3[1]} <= {A_data_out_reg[15:8], r3[2]};
      {r1[0], r2[0], r3[0]} <= {A_data_out_reg[23:16], r2[1], r3[1]};

      c3[2] <= B_data_out_reg[7:0];
      {c2[1], c3[1]} <= {B_data_out_reg[15:8], c3[2]};
      {c1[0], c2[0], c3[0]} <= {B_data_out_reg[23:16], c2[1], c3[1]};
    end else begin
      r1[0] <= 8'd0;
      r2[0] <= 8'd0;
      r2[1] <= 8'd0;
      r3[0] <= 8'd0;
      r3[1] <= 8'd0;
      r3[2] <= 8'd0;
      c1[0] <= 8'd0;
      c2[0] <= 8'd0;
      c2[1] <= 8'd0;
      c3[0] <= 8'd0;
      c3[1] <= 8'd0;
      c3[2] <= 8'd0;
    end
  end
end

// PE interconnect
wire signed [7:0] pe_a_out [0:3][0:3];
wire signed [7:0] pe_b_out [0:3][0:3];
wire signed [31:0] pe_acc [0:3][0:3];
wire clear_acc0, clear_acc1, clear_acc2, clear_acc3;

assign clear_acc0 = ((control == 4'd8) || in_valid == 1);
assign clear_acc1 = ((control == 4'd9) || in_valid == 1);
assign clear_acc2 = ((control == 4'd10) || in_valid == 1);
assign clear_acc3 = ((control == 4'd11) || in_valid == 1);

// PE
PE pe_00(clk, rst_n, busy, clear_acc0, input_offset, A_data_out_reg[31:24], pe_a_out[0][0], B_data_out_reg[31:24], pe_b_out[0][0], pe_acc[0][0]);
PE pe_01(clk, rst_n, busy, clear_acc0, input_offset, pe_a_out[0][0], pe_a_out[0][1], c1[0], pe_b_out[0][1], pe_acc[0][1]);
PE pe_02(clk, rst_n, busy, clear_acc0, input_offset, pe_a_out[0][1], pe_a_out[0][2], c2[0], pe_b_out[0][2], pe_acc[0][2]);
PE pe_03(clk, rst_n, busy, clear_acc0, input_offset, pe_a_out[0][2], pe_a_out[0][3], c3[0], pe_b_out[0][3], pe_acc[0][3]);

PE pe_10(clk, rst_n, busy, clear_acc1, input_offset, r1[0], pe_a_out[1][0], pe_b_out[0][0], pe_b_out[1][0], pe_acc[1][0]);
PE pe_11(clk, rst_n, busy, clear_acc1, input_offset, pe_a_out[1][0], pe_a_out[1][1], pe_b_out[0][1], pe_b_out[1][1], pe_acc[1][1]);
PE pe_12(clk, rst_n, busy, clear_acc1, input_offset, pe_a_out[1][1], pe_a_out[1][2], pe_b_out[0][2], pe_b_out[1][2], pe_acc[1][2]);
PE pe_13(clk, rst_n, busy, clear_acc1, input_offset, pe_a_out[1][2], pe_a_out[1][3], pe_b_out[0][3], pe_b_out[1][3], pe_acc[1][3]);

PE pe_20(clk, rst_n, busy, clear_acc2, input_offset, r2[0], pe_a_out[2][0], pe_b_out[1][0], pe_b_out[2][0], pe_acc[2][0]);
PE pe_21(clk, rst_n, busy, clear_acc2, input_offset, pe_a_out[2][0], pe_a_out[2][1], pe_b_out[1][1], pe_b_out[2][1], pe_acc[2][1]);
PE pe_22(clk, rst_n, busy, clear_acc2, input_offset, pe_a_out[2][1], pe_a_out[2][2], pe_b_out[1][2], pe_b_out[2][2], pe_acc[2][2]);
PE pe_23(clk, rst_n, busy, clear_acc2, input_offset, pe_a_out[2][2], pe_a_out[2][3], pe_b_out[1][3], pe_b_out[2][3], pe_acc[2][3]);

PE pe_30(clk, rst_n, busy, clear_acc3, input_offset, r3[0], pe_a_out[3][0], pe_b_out[2][0], pe_b_out[3][0], pe_acc[3][0]);
PE pe_31(clk, rst_n, busy, clear_acc3, input_offset, pe_a_out[3][0], pe_a_out[3][1], pe_b_out[2][1], pe_b_out[3][1], pe_acc[3][1]);
PE pe_32(clk, rst_n, busy, clear_acc3, input_offset, pe_a_out[3][1], pe_a_out[3][2], pe_b_out[2][2], pe_b_out[3][2], pe_acc[3][2]);
PE pe_33(clk, rst_n, busy, clear_acc3, input_offset, pe_a_out[3][2], pe_a_out[3][3], pe_b_out[2][3], pe_b_out[3][3], pe_acc[3][3]);

// count
reg [7:0] M_reg, M_count, M_tiles, tile_row;
reg [7:0] N_reg, N_tiles, tile_col;
reg [7:0] K_reg, K_count;

always@(in_valid) begin
  if(in_valid) begin
    input_offset = InputOffset;
    M_reg = M;
    N_reg = N;
    K_reg = K;
    M_tiles = (M[1:0] === 2'b00) ? (M >> 2) : (M >> 2) + 1;
    N_tiles = (N[1:0] === 2'b00) ? (N >> 2) : (N >> 2) + 1;
  end
end

always @(posedge clk or negedge rst_n) begin
  if(~rst_n) begin
    a_index <= 0;
    b_index <= 0;
    c_index <= 0;
    K_count <= 1;
    tile_row <= 0;
    tile_col <= 0;
    M_count <= 0;
    stop <= 0;
    c_cnt <= 0;
  end else begin
    if(busy) begin
      case (control)
        4'd0, 4'd1, 4'd2, 4'd3: begin
          K_count <= K_count + 1;
          if(K_count > K_reg) begin
            a_index <= a_index;
            b_index <= b_index;
          end else begin
            a_index <= a_index + 1;
            b_index <= b_index + 1;
          end
        end
        4'd4, 4'd5, 4'd6, 4'd7: begin
          K_count <= 1;
        end
        4'd8, 4'd9, 4'd10: begin
          c_cnt <= c_cnt + 1;
          M_count <= M_count + 1;
          if(M_count < M_reg) begin
            c_index <= c_index + 1;
          end
        end
        4'd11: begin
          c_cnt <= 0;
          M_count <= M_count + 1;
          if(M_count < M_reg) begin
            c_index <= c_index + 1;
          end
          if(tile_row == (M_tiles - 1)) begin
            if(tile_col == (N_tiles - 1)) begin
              stop <= 1;
            end else begin
              tile_col <= tile_col + 1;
              M_count <= 0;
              tile_row <= 0;
              b_index <= b_index;
              a_index <= 0;
            end
          end else begin
            tile_row <= tile_row + 1;
            a_index <= a_index;
            b_index <= tile_col*K_reg;
          end
        end
      endcase
    end else begin
      a_index <= 0;
      b_index <= 0;
      c_index <= 0;
      K_count <= 1;
      tile_row <= 0;
      tile_col <= 0;
      M_count <= 0;
      stop <= 0;
    end
  end
end


// control
always @(posedge clk or negedge rst_n) begin
  if(~rst_n) begin
    control <= 0;
  end else if(busy) begin
    if(control == 4'd11) begin
      control <= 0;
    end else if(control == 4'd3) begin
      control <= (K_count < K_reg) ? 4'd3 : (control+1);
    end else begin
      control <= control + 1;
    end
  end
end

assign C_wr_en = ((control >= 8 && control <= 11) && (M_count < M_reg)) ? 1 : 0;
always @(*) begin
  if (~rst_n) begin
    A_data_out_reg = 32'd0;
    B_data_out_reg = 32'd0;
  end else begin
    if(busy) begin
      case (control)
        4'd0, 4'd1, 4'd2, 4'd3: begin
          if(K_count > K_reg) begin
            A_data_out_reg = 32'd0;
            B_data_out_reg = 32'd0;
          end else begin
            A_data_out_reg = A_data_out;
            B_data_out_reg = B_data_out;
          end
        end
        default: begin
          A_data_out_reg = 32'd0;
          B_data_out_reg = 32'd0;
        end
      endcase
    end else begin
      A_data_out_reg = 32'd0;
      B_data_out_reg = 32'd0;
    end
  end
end

endmodule