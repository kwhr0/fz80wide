// ---------------------------------------------------------------------
// File name         : testpattern.v
// Module name       : testpattern
// Created by        : Caojie
// Module Description: 
//						I_mode[2:0] = "000" : color bar     
//						I_mode[2:0] = "001" : net grid     
//						I_mode[2:0] = "010" : gray         
//						I_mode[2:0] = "011" : single green
//                      I_mode[2:0] = "100" : single blue
//                      I_mode[2:0] = "101" : single red
//                
// ---------------------------------------------------------------------
// Release history
// VERSION |   Date      | AUTHOR  |    DESCRIPTION
// --------------------------------------------------------------------
//   1.0   | 24-Sep-2009 | Caojie  |    initial
// --------------------------------------------------------------------

module testpattern
(
	input cpu_clk,
	input mw_l, mw_u, iowr,
	input [12:0] cpu_adr,
	input [15:0] cpu_data,
	output reg vrtc,

	input              I_pxl_clk   ,//pixel clock
    input              I_rst_n     ,//low active 
    output             O_de        ,   
    output reg         O_hs        ,
    output reg         O_vs        ,
    output     [7:0]   O_data_r    ,    
    output     [7:0]   O_data_g    ,
    output     [7:0]   O_data_b    
); 

//====================================================
// VESA standard 848x480 V:60Hz H:31kHz pix:33.75MHz

localparam I_h_total  = 12'd1088; //hor total time
localparam I_h_sync   = 12'd112;  //hor sync time
localparam I_h_bporch = 12'd112;  //hor back porch
localparam I_h_res    = 12'd848;  //hor resolution
localparam I_v_total  = 12'd517;  //ver total time
localparam I_v_sync   = 12'd8;    //ver sync time
localparam I_v_bporch = 12'd23;   //ver back porch
localparam I_v_res    = 12'd480;  //ver resolution

localparam N = 5; //delay N clocks

//====================================================
reg  [11:0]   V_cnt     ;
reg  [11:0]   H_cnt     ;
              
wire          Pout_de_w    ;                          
wire          Pout_hs_w    ;
wire          Pout_vs_w    ;

reg  [N-1:0]  Pout_de_dn   ;                          
reg  [N-1:0]  Pout_hs_dn   ;
reg  [N-1:0]  Pout_vs_dn   ;

//----------------------------
wire 		  De_pos;
wire 		  De_neg;
wire 		  Vs_pos;
	
reg  [11:0]   De_vcnt     ;
reg  [11:0]   De_hcnt     ;

//-------------------------------
reg  [23:0]   Data_tmp/*synthesis syn_keep=1*/;

//==============================================================================
//Generate HS, VS, DE signals
always@(posedge I_pxl_clk or negedge I_rst_n)
begin
	if(!I_rst_n)
		V_cnt <= 12'd0;
	else     
		begin
			if((V_cnt >= (I_v_total-1'b1)) && (H_cnt >= (I_h_total-1'b1)))
				V_cnt <= 12'd0;
			else if(H_cnt >= (I_h_total-1'b1))
				V_cnt <=  V_cnt + 1'b1;
			else
				V_cnt <= V_cnt;
		end
end

//-------------------------------------------------------------    
always @(posedge I_pxl_clk or negedge I_rst_n)
begin
	if(!I_rst_n)
		H_cnt <=  12'd0; 
	else if(H_cnt >= (I_h_total-1'b1))
		H_cnt <=  12'd0 ; 
	else 
		H_cnt <=  H_cnt + 1'b1 ;           
end

//-------------------------------------------------------------
assign  Pout_de_w = ((H_cnt>=(I_h_sync+I_h_bporch))&(H_cnt<=(I_h_sync+I_h_bporch+I_h_res-1'b1)))&
                    ((V_cnt>=(I_v_sync+I_v_bporch))&(V_cnt<=(I_v_sync+I_v_bporch+I_v_res-1'b1))) ;
assign  Pout_hs_w =  ~((H_cnt>=12'd0) & (H_cnt<=(I_h_sync-1'b1))) ;
assign  Pout_vs_w =  ~((V_cnt>=12'd0) & (V_cnt<=(I_v_sync-1'b1))) ;  

//-------------------------------------------------------------
always@(posedge I_pxl_clk or negedge I_rst_n)
begin
	if(!I_rst_n)
		begin
			Pout_de_dn  <= {N{1'b0}};                          
			Pout_hs_dn  <= {N{1'b1}};
			Pout_vs_dn  <= {N{1'b1}}; 
		end
	else 
		begin
			Pout_de_dn  <= {Pout_de_dn[N-2:0],Pout_de_w};                          
			Pout_hs_dn  <= {Pout_hs_dn[N-2:0],Pout_hs_w};
			Pout_vs_dn  <= {Pout_vs_dn[N-2:0],Pout_vs_w}; 
		end
end

assign O_de = Pout_de_dn[4];

always@(posedge I_pxl_clk or negedge I_rst_n)
begin
	if(!I_rst_n)
		begin                        
			O_hs  <= 1'b1;
			O_vs  <= 1'b1; 
		end
	else 
		begin                         
			O_hs  <= ~Pout_hs_dn[3];
			O_vs  <= ~Pout_vs_dn[3];
		end
end

//=================================================================================
assign De_pos	= !Pout_de_dn[1] & Pout_de_dn[0]; //de rising edge
assign De_neg	= Pout_de_dn[1] && !Pout_de_dn[0];//de falling edge
assign Vs_pos	= !Pout_vs_dn[1] && Pout_vs_dn[0];//vs rising edge

always @(posedge I_pxl_clk or negedge I_rst_n)
begin
	if(!I_rst_n)
		De_hcnt <= 12'd0;
	else if (De_pos == 1'b1)
		De_hcnt <= 12'd0;
	else if (Pout_de_dn[1] == 1'b1)
		De_hcnt <= De_hcnt + 1'b1;
	else
		De_hcnt <= De_hcnt;
end

always @(posedge I_pxl_clk or negedge I_rst_n)
begin
	if(!I_rst_n) 
		De_vcnt <= 12'd0;
	else if (Vs_pos == 1'b1)
		De_vcnt <= 12'd0;
	else if (De_neg == 1'b1) begin
		if (~vvalid | chlast) chcnt <= 0;
		else chcnt <= chcnt + 1;
		De_vcnt <= De_vcnt + 1'b1;
	end
	else
		De_vcnt <= De_vcnt;
end

/// DMAC

reg [1:0] dma_seq;
reg [12:0] start_adr;
always @(posedge cpu_clk) begin
	if (~I_rst_n) begin
		dma_seq <= 0;
		start_adr <= 13'h1300; // f300
	end
	else if (iowr & cpu_adr[7:4] == 4'h6) begin
		if (cpu_adr[3:0] == 4'h8 & cpu_data[7:0] == 8'hc4)
			dma_seq <= 1;
		if (cpu_adr[3:0] == 4'h6) begin
			if (dma_seq == 1) begin
				dma_seq <= 2;
				start_adr <= { start_adr[12:8], cpu_data[7:0] };
			end
			if (dma_seq == 2) begin
				dma_seq <= 0;
				start_adr <= { cpu_data[4:0], start_adr[7:0] };
			end
		end
	end
end

/// CRTC

localparam H_START = 8 * 13;
localparam V_START = 32;
localparam WIDTH   = 640;
localparam HEIGHT  = 400;

wire [9:0] dotcnt = De_hcnt[9:0];
reg [4:0] chcnt = 0; // LSB for line doubler
reg [3:0] lpc = 7; // 9 for width ,20
wire chlast = chcnt == { lpc, 1'b1 };
reg width80 = 1;
reg colormode = 1;
reg [2:0] color;
reg [5:0] atr_adr;
reg [6:0] atr = 7'b1110000, atr0 = 7'b1110000;
reg [14:0] atr_data = 0;
reg [4:0] ycnt = 0;
reg [7:0] chrline;
wire [7:0] text_data;
wire [7:0] chrline_c;
wire [6:0] rowbuf_adr = dotcnt[2] ? text_adr : { atr_adr, dotcnt[1] };
wire [7:0] trans_data0, trans_data1;

wire transfer = !chcnt & H_cnt < 120; // rowbuf: 120bytes
reg [12:0] vram_adr;

always @(posedge I_pxl_clk) begin
	if (~vvalid)
		vram_adr <= start_adr;
	else if (transfer) begin
		vram_adr <= vram_adr + 1;
	end
end

vram vram0(
        .dout(trans_data0), //output [7:0] dout
        .clka(cpu_clk), //input clka
        .cea(cpu_adr[0] ? mw_u : mw_l), //input cea
        .reseta(~I_rst_n), //input reseta
        .clkb(I_pxl_clk), //input clkb
        .ceb(1'b1), //input ceb
        .resetb(~I_rst_n), //input resetb
        .oce(1'b1), //input oce
        .ada(cpu_adr[12:1] + cpu_adr[0]),
        .din(cpu_adr[0] ? cpu_data[15:8] : cpu_data[7:0]),
        .adb(vram_adr[12:1])
    );
vram vram1(
        .dout(trans_data1), //output [7:0] dout
        .clka(cpu_clk), //input clka
        .cea(cpu_adr[0] ? mw_l : mw_u), //input cea
        .reseta(~I_rst_n), //input reseta
        .clkb(I_pxl_clk), //input clkb
        .ceb(1'b1), //input ceb
        .resetb(~I_rst_n), //input resetb
        .oce(1'b1), //input oce
        .ada(cpu_adr[12:1]),
        .din(cpu_adr[0] ? cpu_data[7:0] : cpu_data[15:8]),
        .adb(vram_adr[12:1])
    );

reg [6:0] H_cnt1;
always @(posedge I_pxl_clk)
	H_cnt1 <= H_cnt;

rowbuf rowbuf(
        .dout(text_data), //output [7:0] dout
        .clka(I_pxl_clk), //input clka
        .cea(transfer), //input cea
        .reseta(~I_rst_n), //input reseta
        .clkb(I_pxl_clk), //input clkb
        .ceb(1'b1), //input ceb
        .resetb(~I_rst_n), //input resetb
        .oce(1'b1), //input oce
        .ada({ 4'b0000, H_cnt1 }), //input [10:0] ada
	// trans_data is inverted because of delay 1 clock
        .din(vram_adr[0] ? trans_data0 : trans_data1), //input [7:0] din
    //  .din(vram_adr[0] ? trans_data1 : trans_data0), //input [7:0] din
        .adb({ 4'b0000, rowbuf_adr }) //input [10:0] adb
    );

wire hvalid = De_hcnt >= H_START & De_hcnt < H_START + WIDTH;
wire vvalid = De_vcnt >= V_START & De_vcnt < V_START + HEIGHT;
always @(posedge I_pxl_clk) vrtc <= ~vvalid;

// text

reg [6:0] text_adr = 0;
always @(posedge I_pxl_clk) begin
	if (!De_hcnt)
		text_adr <= 0;
	else if (hvalid & dotcnt[2:0] == 3'b111)
		text_adr <= text_adr + 1;
end

// attribute

always @(posedge I_pxl_clk) begin
	if (dotcnt[2:0] == 3'b001) atr_data[14:8] <= text_data[6:0];
	if (dotcnt[2:0] == 3'b011) atr_data[7:0] <= text_data;
	if (hvalid & dotcnt[2:0] == 3'b110 & atr_data[14:8] == text_adr) begin
		atr_adr <= atr_adr + 1;
		if (colormode & atr_data[3]) atr[6:3] <= atr_data[7:4];
		else atr[2:0] <= atr_data[2:0];
		if (~colormode) atr[6:3] <= { 3'b111, atr_data[7] };
	end
	if (De_hcnt == H_START + WIDTH - 1) begin
		if (De_vcnt == 32 - 1) begin
			atr_adr <= 6'h28;
			ycnt <= 0;
		end
		else if (chlast) begin
			atr_adr <= 6'h28;
			atr0 <= atr;
			ycnt <= ycnt + 1;
		end
		else begin
			atr_adr <= 6'h28;
			atr <= atr0;
		end
	end
end

///

font8x8 font(.dout(chrline_c), .clk(I_pxl_clk), .oce(1'b1), .ce(1'b1), .reset(~I_rst_n),
            .ad({ text_data, chcnt[3:1] }));
wire [3:0] text_data_l = text_data[3:0];
wire [3:0] text_data_u = text_data[7:4];
wire dotl = text_data_l[chcnt[3:2]];
wire dotr = text_data_u[chcnt[3:2]];
wire [7:0] chrline_g = { dotl, dotl, dotl, dotl, dotr, dotr, dotr, dotr };
always @(posedge I_pxl_clk) begin
	if (dotcnt[2:0] == 3'b111 & (width80 | ~dotcnt[3])) begin
		if (hvalid & vvalid & ~chcnt[4])
			chrline <= atr[3] ? chrline_g : chrline_c;
		else chrline <= 8'b00000000;
		color <= atr[6:4];
	end
	else if (width80 | dotcnt[0]) chrline <= chrline << 1;
end

wire [23:0] color_data = { {8{color[0]}}, {8{color[2]}}, {8{color[1]}} };
always @(posedge I_pxl_clk or negedge I_rst_n)
begin
	if(!I_rst_n)
		Data_tmp <= 24'd0;
	else if(Pout_de_dn[2] == 1'b1)
		Data_tmp <= chrline[7] ? color_data : 24'b0;
	else
		Data_tmp <= 24'd0;
end

assign O_data_r = Data_tmp[ 7: 0];
assign O_data_g = Data_tmp[15: 8];
assign O_data_b = Data_tmp[23:16];

endmodule       
              
