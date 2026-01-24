`define COMCLK
//`define USEDIV

module top(
	input clk,
	input rst_n,
	output tmds_clk_n_0,
	output tmds_clk_p_0,
	output [2:0] tmds_d_n_0,
	output [2:0] tmds_d_p_0,
	output pa_en,
	output bclk,
	output lrclk,
	output sout
);

localparam WAIT_EN	= 0;
localparam INT_EN	= 1;

function [7:0] sel4x8;
	input [1:0] sel;
	input [31:0] a;
	begin
		case (sel)
			2'b00: sel4x8 = a[7:0];
			2'b01: sel4x8 = a[15:8];
			2'b10: sel4x8 = a[23:16];
			2'b11: sel4x8 = a[31:24];
		endcase
	end
endfunction

wire [15:0] pc, adr, data_in, data_out;

wire [13:0] iadr = pc[15:2], iadr_next = iadr + 1;
wire [13:0] iadr0 = |pc[1:0] ? iadr_next : iadr;
wire [13:0] iadr1 =  pc[1]   ? iadr_next : iadr;
wire [13:0] iadr2 = &pc[1:0] ? iadr_next : iadr;
wire [13:0] iadr3 = iadr;
wire [31:0] insn;

wire [13:0] dadr = adr[15:2], dadr_next = dadr + 1;
wire [13:0] dadr0 = &adr[1:0] ? dadr_next : dadr;

wire mw_l, mw_u;
wire we0 = mw_l & adr[1:0] == 2'b00 | mw_u & adr[1:0] == 2'b11;
wire we1 = mw_l & adr[1:0] == 2'b01 | mw_u & adr[1:0] == 2'b00;
wire we2 = mw_l & adr[1:0] == 2'b10 | mw_u & adr[1:0] == 2'b01;
wire we3 = mw_l & adr[1:0] == 2'b11 | mw_u & adr[1:0] == 2'b10;

wire [7:0] dl = data_out[7:0], du = data_out[15:8];
wire [7:0] ramd0, ramd1, ramd2, ramd3;

wire cpu_clk;

reg [7:0] sel_adr;
always @(posedge cpu_clk)
	sel_adr <= adr[7:0];

wire iord, iowr;
wire in40 = iord & sel_adr[7:4] == 4'b0100;

wire vrtc;
wire [7:0] in_data = in40 ? { 2'b00, vrtc, 5'b11010 } : 8'hff; // RX

assign data_in = {
	sel4x8(sel_adr, { ramd0, ramd3, ramd2, ramd1 }),
	iord ? in_data : sel4x8(sel_adr, { ramd3, ramd2, ramd1, ramd0 })
};

ram #(.FILE("ram0.mem"))
	ram0(.clk(cpu_clk), .ada(iadr0), .douta(insn[7:0]),
	.adb(dadr0), .dinb(adr[0] ? du : dl), .doutb(ramd0), .wreb(we0));
ram #(.FILE("ram1.mem"))
	ram1(.clk(cpu_clk), .ada(iadr1), .douta(insn[15:8]),
	.adb(dadr),  .dinb(adr[0] ? dl : du), .doutb(ramd1), .wreb(we1));
ram #(.FILE("ram2.mem"))
	ram2(.clk(cpu_clk), .ada(iadr2), .douta(insn[23:16]),
	.adb(dadr),  .dinb(adr[0] ? du : dl), .doutb(ramd2), .wreb(we2));
ram #(.FILE("ram3.mem"))
	ram3(.clk(cpu_clk), .ada(iadr3), .douta(insn[31:24]),
	.adb(dadr),  .dinb(adr[0] ? dl : du), .doutb(ramd3), .wreb(we3));

reg intreq;
wire intack;
reg vrtc1;
always @(posedge cpu_clk)
	vrtc1 <= vrtc;
always @(posedge cpu_clk)
	if (!rst_n) intreq <= 1'b0;
	else if (intack) intreq <= 1'b0;
	else if (INT_EN & vrtc & ~vrtc1) intreq <= 1'b1;

wire flip;
reg flip1;
always @(posedge cpu_clk)
	flip1 <= flip;
wire start = flip ^ flip1;

reg [3:0] waitcnt;
always @(posedge cpu_clk or posedge start)
	if (start) waitcnt <= ~0;
	else waitcnt <= waitcnt - 1'b1;
wire waitreq = WAIT_EN & |waitcnt;

fz80wide cpu(.clk(cpu_clk), .reset(~hdmi4_rst_n), .pc_out(pc), .insn_in(insn),
	.adr_out(adr), .data_in(data_in), .data_out(data_out),
	.mw_l(mw_l), .mw_u(mw_u), .iord(iord), .iowr(iowr), .flip(flip),
	.waitreq(waitreq), .intreq(intreq), .intack(intack),
	.nmireq(1'b0), .busreq(1'b0));

//

wire tp0_vs_in, tp0_hs_in, tp0_de_in;
wire [7:0] tp0_data_r, tp0_data_g, tp0_data_b;

testpattern testpattern(
	.cpu_clk(cpu_clk),
	.mw_l(mw_l & adr[15:13] == 3'b111),
	.mw_u(mw_u & adr[15:13] == 3'b111),
	.iowr(iowr & ~waitreq),
	.cpu_adr(adr[12:0]),
	.cpu_data(data_out),
	.vrtc(vrtc),

	.I_pxl_clk   (video_clk),
	.I_rst_n     (rst_n),
	.O_de        (tp0_de_in),   
	.O_hs        (tp0_hs_in),
	.O_vs        (tp0_vs_in),
	.O_data_r    (tp0_data_r),   
	.O_data_g    (tp0_data_g),
	.O_data_b    (tp0_data_b)
);

//

wire serial_clk, tmp_clk;
wire vesa_pll_lock, cpu_pll_lock;

`ifdef COMCLK
assign cpu_clk = video_clk;
assign cpu_pll_lock = 1'b1;
`else
`ifdef USEDIV
CLKDIV clkdiv1(.RESETN(hdmi4_rst_n), .CALIB(1'b0), .HCLKIN(clk), .CLKOUT(tmp_clk));
defparam clkdiv1.DIV_MODE = "8";
CLKDIV clkdiv2(.RESETN(hdmi4_rst_n), .CALIB(1'b0), .HCLKIN(tmp_clk), .CLKOUT(cpu_clk));
defparam clkdiv2.DIV_MODE = "8";
assign cpu_pll_lock = 1'b1;
`else
cpu_pll cpu_pll(.clkin(clk), .mdclk(clk), .clkout0(cpu_clk), .lock(cpu_pll_lock));
`endif
`endif

vesa_pll vesa_pll(.clkin(clk), .mdclk(clk), .clkout0(serial_clk), .lock(vesa_pll_lock));
CLKDIV vesa_clkdiv(.RESETN(hdmi4_rst_n), .CALIB(1'b0), .HCLKIN(serial_clk), .CLKOUT(video_clk));
defparam vesa_clkdiv.DIV_MODE = "5";

assign hdmi4_rst_n = rst_n & vesa_pll_lock & cpu_pll_lock;

DVI_TX_Top DVI_TX_Top(
	.I_rst_n       (hdmi4_rst_n),
	.I_serial_clk  (serial_clk),

	.I_rgb_clk     (video_clk),
	.I_rgb_vs      (tp0_vs_in), 
	.I_rgb_hs      (tp0_hs_in),    
	.I_rgb_de      (tp0_de_in), 
	.I_rgb_r       (tp0_data_r), 
	.I_rgb_g       (tp0_data_g),  
	.I_rgb_b       (tp0_data_b),  

	.O_tmds_clk_p  (tmds_clk_p_0),
	.O_tmds_clk_n  (tmds_clk_n_0),
	.O_tmds_data_p (tmds_d_p_0),
	.O_tmds_data_n (tmds_d_n_0)
);

//

wire [15:0] tone;
dcsg dcsg(.clk(cpu_clk), .we(iowr & adr[7:4] == 4'b1001), .data(data_out[7:0]), .sound_out(tone));

i2s i2s(.clk(cpu_clk), .sound_in(tone), .bclk(bclk), .lrclk(lrclk), .sout(sout));
assign pa_en = 1'b1;

endmodule
