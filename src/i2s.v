`define I2S_DIV		17

module i2s(clk, sound_in, bclk, lrclk, sout);
input clk;
input [15:0] sound_in;
output bclk, lrclk, sout;

reg [4:0] divcnt = 0;
reg [5:0] cnt = 0;
reg [15:0] data = 0;
assign bclk = cnt[0];
assign lrclk = cnt[5];
assign sout = data[15];

always @(posedge clk)
	if (|divcnt) divcnt <= divcnt - 1'b1;
	else begin
		divcnt <= `I2S_DIV - 1'b1;
		if (cnt[0]) data <= |cnt[4:1] ? { data[14:0], 1'b0 } : sound_in;
		cnt <= cnt + 1'b1;
	end

endmodule
