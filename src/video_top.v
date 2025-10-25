//`define COMCLK

// ==============0ooo===================================================0ooo===========
// =  Copyright (C) 2014-2020 Gowin Semiconductor Technology Co.,Ltd.
// =                     All rights reserved.
// ====================================================================================
// 
//  __      __      __
//  \ \    /  \    / /   [File name   ] video_top.v
//   \ \  / /\ \  / /    [Description ] Video demo
//    \ \/ /  \ \/ /     [Timestamp   ] Friday April 10 14:00:30 2020
//     \  /    \  /      [version     ] 2.0
//      \/      \/
//
// ==============0ooo===================================================0ooo===========
// Code Revision History :
// ----------------------------------------------------------------------------------
// Ver:    |  Author    | Mod. Date    | Changes Made:
// ----------------------------------------------------------------------------------
// V1.0    | Caojie     |  4/10/20     | Initial version 
// ----------------------------------------------------------------------------------
// V2.0    | Caojie     | 10/30/20     | DVI IP update 
// ----------------------------------------------------------------------------------
// ==============0ooo===================================================0ooo===========

module video_top
(
    input             I_clk           , //27Mhz
    input             I_rst           ,
    input             I_key           ,
    output     [4:0]  O_led           ,
    output            running         ,
    output            O_tmds_clk_p    ,
    output            O_tmds_clk_n    ,
    output     [2:0]  O_tmds_data_p   ,//{r,g,b}
    output     [2:0]  O_tmds_data_n   
);

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

wire i_en = 1'b1;

ram0 ram0(.clka(cpu_clk), .ada(iadr0), .dina(8'h00), .douta(insn[7:0]),
	.reseta(1'b0), .cea(i_en), .ocea(1'b1), .wrea(1'b0),
	.clkb(cpu_clk), .adb(dadr0), .dinb(adr[0] ? du : dl), .doutb(ramd0),
	.resetb(1'b0), .ceb(1'b1), .oceb(1'b1), .wreb(we0));
ram1 ram1(.clka(cpu_clk), .ada(iadr1), .dina(8'h00), .douta(insn[15:8]),
	.reseta(1'b0), .cea(i_en), .ocea(1'b1), .wrea(1'b0),
	.clkb(cpu_clk), .adb(dadr), .dinb(adr[0] ? dl : du), .doutb(ramd1),
	.resetb(1'b0), .ceb(1'b1), .oceb(1'b1), .wreb(we1));
ram2 ram2(.clka(cpu_clk), .ada(iadr2), .dina(8'h00), .douta(insn[23:16]),
	.reseta(1'b0), .cea(i_en), .ocea(1'b1), .wrea(1'b0),
	.clkb(cpu_clk), .adb(dadr), .dinb(adr[0] ? du : dl), .doutb(ramd2),
	.resetb(1'b0), .ceb(1'b1), .oceb(1'b1), .wreb(we2));
ram3 ram3(.clka(cpu_clk), .ada(iadr3), .dina(8'h00), .douta(insn[31:24]),
	.reseta(1'b0), .cea(i_en), .ocea(1'b1), .wrea(1'b0),
	.clkb(cpu_clk), .adb(dadr), .dinb(adr[0] ? dl : du), .doutb(ramd3),
	.resetb(1'b0), .ceb(1'b1), .oceb(1'b1), .wreb(we3));

reg intreq;
wire intack;
reg vrtc1;
always @(posedge cpu_clk)
	vrtc1 <= vrtc;
always @(posedge cpu_clk)
	if (I_rst) intreq <= 1'b0;
	else if (intack) intreq <= 1'b0;
//	else if (vrtc & ~vrtc1) intreq <= 1'b1;

fz80wide cpu(.clk(cpu_clk), .reset(I_rst), .pc_out(pc), .insn_in(insn),
	.adr_out(adr), .data_in(data_in), .data_out(data_out),
	.mw_l(mw_l), .mw_u(mw_u), .iord(iord), .iowr(iowr),
	.intreq(intreq), .intack(intack), .nmireq(1'b0), .busreq(1'b0));

//==================================================
wire      I_rst_n = ~I_rst;

reg  [1:0]  KEY_sync;

reg  [31:0] run_cnt;

//KEY Switch MODE
always @(posedge pix_clk or negedge hdmi4_rst_n) begin
    if (!hdmi4_rst_n)    
        KEY_sync <= 2'b00;
    else                 
        KEY_sync <= {KEY_sync[0], I_key};
end


wire KEY_pressed =  KEY_sync[0] & ~KEY_sync[1];

//--------------------------
wire        tp0_vs_in  ;
wire        tp0_hs_in  ;
wire        tp0_de_in ;
wire [ 7:0] tp0_data_r/*synthesis syn_keep=1*/;
wire [ 7:0] tp0_data_g/*synthesis syn_keep=1*/;
wire [ 7:0] tp0_data_b/*synthesis syn_keep=1*/;

reg         vs_r;
reg  [9:0]  cnt_vs;

//------------------------------------
//HDMI4 TX
wire serial_clk;
wire pll_lock;

wire hdmi4_rst_n;

wire pix_clk;

//===================================================
//LED test
always @(posedge I_clk or negedge I_rst_n) //I_clk
begin
    if(!I_rst_n)
        run_cnt <= 32'd0;
    else if(run_cnt >= 32'd27_000_000)
        run_cnt <= 32'd0;
    else
        run_cnt <= run_cnt + 1'b1;
end

assign  running = (run_cnt < 32'd14_000_000) ? 1'b1 : 1'b0;

//===========================================================================
testpattern testpattern_inst
(
	.cpu_clk(cpu_clk),
	.mw_l(mw_l & adr[15:13] == 3'b111),
	.mw_u(mw_u & adr[15:13] == 3'b111),
	.iowr(iowr),
	.cpu_adr(adr[12:0]),
	.cpu_data(data_out),
	.vrtc(vrtc),
    .I_pxl_clk   (pix_clk            ),//pixel clock
    .I_rst_n     (hdmi4_rst_n        ),//low active 
    .O_de        (tp0_de_in          ),   
    .O_hs        (tp0_hs_in          ),
    .O_vs        (tp0_vs_in          ),
    .O_data_r    (tp0_data_r         ),   
    .O_data_g    (tp0_data_g         ),
    .O_data_b    (tp0_data_b         )
);

always@(posedge pix_clk)
begin
    vs_r<=tp0_vs_in;
end

always@(posedge pix_clk or negedge hdmi4_rst_n)
begin
    if(!hdmi4_rst_n)
        cnt_vs<=0;
    else if(vs_r && !tp0_vs_in) //vKEY4 falling edge
        cnt_vs<=cnt_vs+1'b1;
end 

//==============================================================================
//TMDS TX(HDMI4)
TMDS_rPLL u_tmds_rpll
(.clkin     (I_clk     )     //input clk 
,.clkout    (serial_clk)     //output clk 
,.lock      (pll_lock  )     //output lock
);

`ifdef COMCLK
assign cpu_clk = I_clk;
`else
wire clk_a, clk_b;

CLKDIV clkdiv_a(.RESETN(I_rst_n), .HCLKIN(I_clk), .CALIB(1'b0), .CLKOUT(cpu_clk));
defparam clkdiv_a.DIV_MODE="2";
defparam clkdiv_a.GSREN="false";
/*
CLKDIV clkdiv_b(.RESETN(I_rst_n), .HCLKIN(clk_a), .CALIB(1'b0), .CLKOUT(clk_b));
defparam clkdiv_b.DIV_MODE="4";
defparam clkdiv_b.GSREN="false";

CLKDIV clkdiv_c(.RESETN(I_rst_n), .HCLKIN(clk_b), .CALIB(1'b0), .CLKOUT(cpu_clk));
defparam clkdiv_c.DIV_MODE="4";
defparam clkdiv_c.GSREN="false";
*/
`endif

assign hdmi4_rst_n = I_rst_n & pll_lock;

CLKDIV u_clkdiv
(.RESETN(hdmi4_rst_n)
,.HCLKIN(serial_clk) //clk  x5
,.CLKOUT(pix_clk)    //clk  x1
,.CALIB (1'b1)
);
defparam u_clkdiv.DIV_MODE="5";
defparam u_clkdiv.GSREN="false";

DVI_TX_Top DVI_TX_Top_inst
(
    .I_rst_n       (hdmi4_rst_n   ),  //asynchronous reset, low active
    .I_serial_clk  (serial_clk    ),
    .I_rgb_clk     (pix_clk       ),  //pixel clock
    .I_rgb_vs      (tp0_vs_in     ), 
    .I_rgb_hs      (tp0_hs_in     ),    
    .I_rgb_de      (tp0_de_in     ), 
    .I_rgb_r       (  tp0_data_r ),  //tp0_data_r
    .I_rgb_g       (  tp0_data_g  ),  
    .I_rgb_b       (  tp0_data_b  ),  
    .O_tmds_clk_p  (O_tmds_clk_p  ),
    .O_tmds_clk_n  (O_tmds_clk_n  ),
    .O_tmds_data_p (O_tmds_data_p ),  //{r,g,b}
    .O_tmds_data_n (O_tmds_data_n )
);

key_led_ctrl key_led_ctrl_inst
(
    .I_rst_n       (hdmi4_rst_n   ),
    .I_clk         (pix_clk       ), 
    .I_key         (I_key         ),
    .O_led         (O_led         )
);

endmodule
