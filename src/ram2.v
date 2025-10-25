module ram2(clka, ada, dina, douta, reseta, cea, ocea, wrea, clkb, adb, dinb, doutb, resetb, ceb, oceb, wreb);
input clka, reseta, cea, ocea, wrea, clkb, resetb, ceb, oceb, wreb;
input [13:0] ada, adb;
input [7:0] dina, dinb;
output [7:0] douta, doutb;
reg [7:0] douta, doutb;
reg [7:0] mem[0:'h3fff];
initial $readmemh("ram2.mem", mem);
always @(posedge clka)
	if (cea & wrea) mem[ada] <= dina;
always @(posedge clkb)
	if (ceb & wreb) mem[adb] <= dinb;
always @(posedge clka)
	if (cea & wrea) douta <= dina;
	else if (ceb & wreb & ada == adb) douta <= dinb;
	else douta <= mem[ada];
always @(posedge clkb)
	if (ceb & wreb) doutb <= dinb;
	else if (cea & wrea & ada == adb) doutb <= dina;
	else doutb <= mem[adb];
endmodule
