// Z80 CPU binary compatible soft core
// 32-bit instruction bus & 16-bit data bus
// Copyright 2025 © Yasuo Kuwahara

// MIT License

// not implemented: IM2, WAIT

module fz80wide(clk, reset, pc_out, insn_in, adr_out, data_in, data_out,
	mw_l, mw_u, iord, iowr, intreq, intack, nmireq, nmiack, busreq, busack);
input clk, reset, intreq, nmireq, busreq;
input [31:0] insn_in;
input [15:0] data_in;
output [15:0] pc_out, adr_out, data_out;
output mw_l, mw_u, iord, iowr, intack, nmiack, busack;

reg [15:0] pc, sp;
reg [7:0] rb[0:1], rc[0:1], rd[0:1], re[0:1], rh[0:1], rl[0:1];
reg [7:0] ra[0:1], rf[0:1];
reg [7:0] ixl, ixh, iyl, iyh, ri, rr;
reg [1:0] intmode;
reg iff1, iff2, intack, nmiack;

reg sel_bcdehl, sel_af;
wire [7:0] b = rb[sel_bcdehl];
wire [7:0] c = rc[sel_bcdehl];
wire [7:0] d = rd[sel_bcdehl];
wire [7:0] e = re[sel_bcdehl];
wire [7:0] h = rh[sel_bcdehl];
wire [7:0] l = rl[sel_bcdehl];
wire [7:0] a = ra[sel_af];
wire [7:0] f = rf[sel_af];
wire [15:0] ix = { ixh, ixl };
wire [15:0] iy = { iyh, iyl };

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

function [7:0] sel8x8;
	input [2:0] sel;
	input [63:0] a;
	begin
		case (sel)
			3'b000: sel8x8 = a[7:0];
			3'b001: sel8x8 = a[15:8];
			3'b010: sel8x8 = a[23:16];
			3'b011: sel8x8 = a[31:24];
			3'b100: sel8x8 = a[39:32];
			3'b101: sel8x8 = a[47:40];
			3'b110: sel8x8 = a[55:48];
			3'b111: sel8x8 = a[63:56];
		endcase
	end
endfunction

function [15:0] sel4x16;
	input [1:0] sel;
	input [63:0] a;
	begin
		case (sel)
			2'b00: sel4x16 = a[15:0];
			2'b01: sel4x16 = a[31:16];
			2'b10: sel4x16 = a[47:32];
			2'b11: sel4x16 = a[63:48];
		endcase
	end
endfunction

// BUS

wire accept = ~(dbl_state & ~state) & ~exec_ret;
reg busack, busack1;
always @(posedge clk) begin
	if (reset | ~busreq) busack <= 1'b0;
	else if (busreq & accept) busack <= 1'b1;
	if (reset) busack1 <= 1'b0;
	else busack1 <= busack;
end
wire clken = ~busack;

reg [31:0] insn_in_l;
reg [15:0] data_in_l;
always @(posedge clk)
	if (~busack1) begin
		insn_in_l <= insn_in;
		data_in_l <= data_in;
	end
wire [31:0] insn_t = busack1 ? insn_in_l : insn_in;
wire [15:0] data = busack1 ? data_in_l : data_in;

//
// DECODE
//

wire force_nop;
wire [7:0] p0 = sel4x8(pc[1:0], insn_t);
wire [7:0] s = sel4x8(pc[1:0], { insn_t[7:0], insn_t[31:8] });
wire multi_pre = &p0[7:6] & p0[4:0] == 5'b11101 & &s[7:6] & s[4:0] == 5'b11101;
wire [7:0] pt = p0 & {8{ ~force_nop & ~multi_pre & ~nmiack }};
wire halt = pt == 8'h76;
wire [7:0] p = pt | {8{ intmode == 2'b10 & intack }};
wire [31:0] insn = {
	sel4x8(pc[1:0], { insn_t[23:0], insn_t[31:24] }),
	sel4x8(pc[1:0], { insn_t[15:0], insn_t[31:16] }),
	s,
	p
};

wire preIndex = &p[7:6] & p[4:0] == 5'b11101;
wire preIndexCB = preIndex & s == 8'hcb;
wire preCB = p == 8'hcb | preIndexCB; // include DDCB/FDCB
wire preED = p == 8'hed;
wire nopre = ~(preCB | preED | preIndex);
wire preNI = nopre | preIndex & s != 8'hcb;
wire indexSw = p[5];

// DD/FD/CB/ED opcode
// DD/FD CB offset opcode
// opcode
wire [7:0] o = preIndexCB ? insn[31:24] : preIndex | preCB | preED ? s : p;

localparam LDSSNN = 0; // ld ss,nn
localparam LDAIR = 1;
localparam LDIA = 2;
localparam LDRA = 3;
localparam PUSHPOP = 4;
localparam EXDEHL = 5;
localparam EXSPHL = 6;
localparam ARITH8CP = 7; // ARITH8,cp r|n
localparam INCDEC8 = 8;
localparam ADD16 = 9;
localparam ADCSBC16 = 10; 
localparam DAA = 11;
localparam DJNZ = 12;
localparam BLOCK = 13;
localparam LDX = 14;
localparam CPX = 15;
localparam IOX = 16;
localparam SRALL = 17;
localparam RLDRRD = 18;
localparam BIT = 19; // bit r|(hl)
localparam CPL = 20;
localparam CCF = 21;
localparam SCF = 22;
localparam RETIN = 23;
localparam EIDI = 24;
localparam IM = 25;
localparam I1MAX = 25;
//
localparam LDRR = 26; // ld r,r'
localparam LDRN = 27; // ld r,n
localparam LDRM = 28; // ld r,(hl|ix|iy)
localparam LDMR = 29; // ld (hl|ix|iy),r
localparam LDMN = 30; // ld (hl|ix|iy),n
localparam LDHL_NN = 31; // ld hl|ix|iy,(nn)
localparam LDSS_NN = 32; // ld ss,(nn)
localparam LDA_BC_A = 33; // ld a,(bc) / ld (bc),a
localparam LDA_DE_A = 34; // ld a,(de) / ld (de),a
localparam LDA_BCDENN = 35; // ld a,(bc|de|nn)
localparam LD_BCDENNA = 36; // ld (bc|de|nn),a
localparam LDHL_NN_HL = 37; // ld (nn),hl|ix|iy|a / ld hl|ix|iy|a,(nn)
localparam LD_NNHL = 38; // ld (nn),hl|ix|iy
localparam LD_NNSS = 39; // ld (nn),bc|de|sp
localparam LDSPHL = 40; // ld sp,hl
localparam PUSH = 41;
localparam POP = 42;
localparam ADDADC8 = 43; // add|adc a,r|(hl|ix|iy)|n
localparam ARITH8 = 44; // add,adc,sub,sbc a,r|n neg
localparam NEG = 45;
localparam INCDEC16 = 46;
localparam LAL8 = 47; // ld,add,adc,sub,sbc,and,xor,or,cp r|(hl|ix|iy)
localparam AND = 48;
localparam XOR = 49;
localparam OR = 50;
localparam LOGIC = 51; // and,or,xor r|n
localparam RA = 52; // rlca,rrca,rla,rra
localparam SRCB = 53; // sll,sla,srl,sra,rlc,rrc,rl,rr r|(hl|ix|iy)
localparam SETRES = 54;
localparam EXCBIT = 55; // except bit r|(hl|ix|iy)
localparam INAN = 56;
localparam INRC = 57;
localparam OUTNA = 58;
localparam OUTCR = 59;
localparam IOCR = 60; // in r,(c) / out (c),r
localparam JP = 61;
localparam JR = 62;
localparam CALL = 63;
localparam RST = 64;
localparam RET = 65;
localparam JPHL = 66;
localparam IMAX = 66;
wire [IMAX:0] i;
assign i[LDRN] = ~|o[7:6] & o[2:0] == 3'b110 & preNI;
assign i[LDHL_NN] = o == 8'h2a & preNI;
assign i[LD_NNHL] = o == 8'h22 & preNI;
assign i[LD_NNSS] = o[7:6] == 2'b01 & o[3:0] == 4'b0011 & preED;
assign i[LDSS_NN] = o[7:6] == 2'b01 & o[3:0] == 4'b1011 & preED;
assign i[LDSPHL] = o == 8'hf9 & preNI;
assign i[PUSHPOP] = &o[7:6] & ~o[3] & o[1:0] == 2'b01 & preNI;
assign i[EXSPHL] = o == 8'he3 & preNI;
assign i[NEG] = preED & s == 8'h44;
assign i[ARITH8] = o[7:5] == 3'b100 & preNI |
	p[7:5] == 3'b110 & p[2:0] == 3'b110 |
	i[NEG];
assign i[ARITH8CP] = o[7:6] == 2'b10 & (~o[5] | &o[4:3]) & preNI |
	p[7:5] == 3'b110 & p[2:0] == 3'b110 |
	i[NEG] |
	p == 8'hfe;
assign i[ADD16] = ~|o[7:6] & o[3:0] == 4'b1001 & preNI;
assign i[ADCSBC16] = s[7:6] == 2'b01 & s[2:0] == 3'b010 & preED;
assign i[INCDEC8] = ~|o[7:6] & o[2:1] == 2'b10 & preNI;
assign i[INCDEC16] = ~|o[7:6] & o[2:0] == 3'b011 & preNI;
assign i[AND] = o[7:3] == 5'b10100 & preNI | p == 8'he6;
assign i[XOR] = o[7:3] == 5'b10101 & preNI | p == 8'hee;
assign i[OR] = o[7:3] == 5'b10110 & preNI | p == 8'hf6;
assign i[LOGIC] = o[7:5] == 3'b101 & ~&o[4:3] & preNI |
	&p[7:5] & ~&p[4:3] & p[2:0] == 3'b110;
assign i[CPL] = p == 8'h2f;
assign i[RLDRRD] = o[7:4] == 4'b0110 & &o[2:0] & preED;
assign i[DJNZ] = p == 8'h10;
assign i[INAN] = p == 8'hdb;
assign i[INRC] = s[7:6] == 2'b01 & ~|s[2:0] & preED;
assign i[OUTNA] = p == 8'hd3;
assign i[OUTCR] = s[7:6] == 2'b01 & s[2:0] == 3'b001 & preED;
assign i[CALL] = p == 8'hcd | &p[7:6] & p[2:0] == 3'b100;
assign i[RST] = &p[7:6] & &p[2:0];
assign i[RET] = p == 8'hc9 | &p[7:6] & ~|p[2:0];
assign i[RETIN] = s[7:4] == 4'b0100 & s[2:0] == 3'b101 & preED;
assign i[DAA] = p == 8'h27;
assign i[BLOCK] = s[7:5] == 3'b101 & ~s[2] & preED;
assign i[LDX] = i[BLOCK] & ~|s[1:0];
assign i[CPX] = i[BLOCK] & s[1:0] == 2'b01;
assign i[IOX] = i[BLOCK] & s[1];
assign i[LDIA] = s == 8'h47 & preED;
assign i[LDRA] = s == 8'h4f & preED;
assign i[RA] = ~|p[7:5] & &p[2:0];
assign i[SRCB] = ~|o[7:6] & preCB;
assign i[SETRES] = o[7] & preCB;
assign i[SRALL] = i[RA] | i[SRCB];
assign i[BIT] = o[7:6] == 2'b01 & preCB;
assign i[EXCBIT] = o[7:6] != 2'b01 & preCB;
assign i[SCF] = p == 8'h37;
assign i[CCF] = p == 8'h3f;
assign i[LDRR] = o[7:6] == 2'b01 & preNI;
assign i[LDSSNN] = ~|o[7:6] & o[3:0] == 4'b0001 & preNI;
assign i[EXDEHL] = p == 8'heb;
assign i[LDRM] = o[7:6] == 2'b01 & o[2:0] == 3'b110 & preNI;
assign i[PUSH] = i[PUSHPOP] & o[2];
assign i[POP] = i[PUSHPOP] & ~o[2];
assign i[LDAIR] = s[7:4] == 4'b0101 & &s[2:0] & preED;
assign i[LDA_BCDENN] = ~|p[7:6] & p[5:4] != 2'b10 & p[3:0] == 4'b1010;
assign i[LD_BCDENNA] = ~|p[7:6] & p[5:4] != 2'b10 & p[3:0] == 4'b0010;
assign i[ADDADC8] = o[7:4] == 4'b1000 & preNI |
	p[7:4] == 4'b1100 & p[2:0] == 3'b110;
assign i[LAL8] = ^o[7:6] & preNI;
assign i[LDMR] = o[7:3] == 5'b01110 & o[2:0] != 3'b110 & preNI;
assign i[LDMN] = o == 8'h36 & preNI;
assign i[LDA_BC_A] = ~|p[7:4] & p[2:0] == 3'b010;
assign i[LDA_DE_A] = p[7:4] == 4'b0001 & p[2:0] == 3'b010;
assign i[IOCR] = s[7:6] == 2'b01 & ~|s[2:1] & preED;
assign i[LDHL_NN_HL] = o[7:5] == 3'b001 & o[2:0] == 3'b010 & preNI;
assign i[JP] = p == 8'hc3 | &p[7:6] & ^p[2:1] & ~p[0];
assign i[JR] = p == 8'h18 | p[7:5] == 3'b001 & ~|p[2:0];
assign i[JPHL] = o == 8'he9 & preNI;
assign i[EIDI] = &p[7:4] & p[2:0] == 3'b011;
assign i[IM] = s[7:5] == 3'b010 & s[2:0] == 3'b110 & preED;

// instruction byte count
// prefix CB		always 2 bytes
// prefix DD/FD CB	always 4 bytes

wire b_prim2 = ~^p[7:6] & p[2:0] == 3'b110 |
	~|p[7:6] & |p[5:4] & ~|p[2:0] |
	p[7:4] == 4'b1101 & p[2:0] == 3'b011;
wire b_prim3 = ~|p[7:6] & p[3:0] == 4'b0001 |
	p[7:5] == 3'b001 & p[2:0] == 3'b010 |
	&p[7:6] & ^p[2:1] & ~p[0] |
	p[7:4] == 4'b1100 & (p[3:1] == 3'b001 | p[3:1] == 3'b110) & p[0];
wire [1:0] b_prim = { b_prim2 | b_prim3, ~b_prim2 };

wire b_index3 = s[2:0] == 3'b110 & ~(~s[7] & s[5:3] == 3'b110) |
	s[7:3] == 5'b01110 & s[2:0] != 3'b110 |
	s[7:1] == 7'b0011010;
wire b_index4 = s == 8'h21 |
	s[7:4] == 4'b0010 & s[2:0] == 3'b010 |
	s == 8'h36;
wire [1:0] b_index = { ~b_index4, b_index3 };

wire [1:0] b_ed = { ~(s[7:6] == 2'b01 & s[2:0] == 3'b011), 1'b0 };

wire [1:0] t_bytes = preCB | preIndex ?
	preNI ? b_index : { ~preIndexCB, 1'b0 } :
	preED ? b_ed : b_prim;
wire [2:0] bytes = { ~|t_bytes[1:0], t_bytes };

// state

wire dbl_state = i[EXSPHL] | i[RLDRRD] | i[LDX] | i[IOX] |
	i[INCDEC8] & o[5:3] == 3'b110 |
	(i[SRCB] | i[SETRES]) & o[2:0] == 3'b110;
reg state;
always @(posedge clk)
	if (reset) state <= 1'b0;
	else if (clken)
		if (state) state <= 1'b0;
		else if (dbl_state) state <= 1'b1;

// PC

wire exit8 = fwd_b == 8'h01;
wire exit16 = ~|fwd_b & fwd_c == 8'h01;
wire [3:0] cond = { fwd_f[7], fwd_f[2], fwd_f[0], fwd_f[6] };
wire cond_ok = cond[p[5:4]] ~^ p[3] | p[0];
wire cond_ok_jr = cond[{ 1'b0, p[4] }] ~^ p[3] | ~p[5];
wire exec_jp = i[JP] & cond_ok;
wire exec_jr = i[JR] & cond_ok_jr | i[DJNZ] & ~exit8;
wire exec_call = i[CALL] & cond_ok;
wire exec_ret = i[RET] & cond_ok | i[RETIN];
wire [15:0] nextpc_normal = pc + bytes;
reg exec_ret1;
always @(posedge clk)
	if (reset) exec_ret1 <= 0;
	else if (clken) exec_ret1 <= exec_ret;
wire [15:0] wd_hl;
wire [15:0] nextpc_imm = nmiack ?  16'h0066 :
	i[RST] ? { insn[5:3], 3'b000 } : insn[23:8];
wire [15:0] nextpc_s = exec_ret1 ? data : nextpc_imm;
wire [15:0] nextpc_rel = nextpc_normal + { {8{ insn[15] }}, insn[15:8] };
wire [15:0] nextpc = exec_jr ? nextpc_rel : i[JPHL] ? wd_hl :
	exec_jp | exec_call | i[RST] | exec_ret1 | nmiack ?
	nextpc_s : nextpc_normal;

wire alu_y_is0;
wire match = i1[CPX] & alu_y_is0;
reg match1;
always @(posedge clk)
	if (reset) match1 <= 1'b0;
	else if (clken) match1 <= match;
wire stay = halt & ~(intack | nmiack) |
	s[4] & ((i[LDX] | i[CPX] & ~match) & ~exit16 | i[IOX] & ~exit8);
reg active;
always @(posedge clk)
	if (reset) active <= 0;
	else if (clken) active <= 1;
assign pc_out = active & ~stay & ~(dbl_state & ~state) ? nextpc : pc;
always @(posedge clk)
	if (reset) pc <= 16'h0;
	else if (clken) pc <= pc_out;

assign force_nop = exec_ret1;

// register bank

always @(posedge clk)
	if (reset) sel_af <= 0;
	else if (clken & p == 8'h08) sel_af <= ~sel_af;
always @(posedge clk)
	if (reset) sel_bcdehl <= 0;
	else if (clken & p == 8'hd9) sel_bcdehl <= ~sel_bcdehl;

// address selector

wire sel_adr_bc = i[LDA_BC_A] | i[IOCR] | i[IOX] & ~s[2] & (s[0] ~^ state);
wire sel_adr_de = i[LDA_DE_A] | i[LDX] & state;
wire sel_adr_sp = i[EXSPHL] | i[PUSHPOP] |
	i[CALL] | i[RST] | i[RET] | i[RETIN] | nmiack;
wire [1:0] sel_adr = { sel_adr_bc | sel_adr_sp, sel_adr_de | sel_adr_sp };

wire sel_adr_imm = i[LDHL_NN_HL] | i[INAN] | i[OUTNA] | i[LDSS_NN] | i[LD_NNSS];

wire [7:0] fwd_b, fwd_c, fwd_d, fwd_e, fwd_h, fwd_l, fwd_f, fwd_a;
wire [7:0] fwd_ixh, fwd_ixl, fwd_iyh, fwd_iyl;
wire [15:0] fwd_ix = { fwd_ixh, fwd_ixl };
wire [15:0] fwd_iy = { fwd_iyh, fwd_iyl };
wire [15:0] fwd_sp;
wire [15:0] disp = { {8{ insn[23] }}, insn[23:16] };
wire [15:0] index_w_ofs = (indexSw ? fwd_iy : fwd_ix) + disp;
wire [15:0] hl_adr = preIndex ? index_w_ofs : { fwd_h, fwd_l };
wire [15:0] sp_adr = { fwd_sp[15:1] - (i[PUSH] | i[CALL] | i[RST] | nmiack), fwd_sp[0] };
wire [15:0] adr_r = sel4x16(sel_adr,
	{ sp_adr, fwd_b, fwd_c, fwd_d, fwd_e, hl_adr });
wire [15:0] imm16_s = nopre ? insn[23:8] : insn[31:16];
assign adr_out = sel_adr_imm ?
	{ i[INAN] | i[OUTNA] ? fwd_a : imm16_s[15:8], imm16_s[7:0] } : adr_r;

// write data (write only)

wire [2:0] sel_wd_r8 = (i[OUTCR] ? o[5:3] : o[2:0]) |
	{3{ ~(i[LDMR] | i[OUTCR]) }};
wire [7:0] wd_r8 = sel8x8(sel_wd_r8,
	{ fwd_a, 8'h00, fwd_l, fwd_h, fwd_e, fwd_d, fwd_c, fwd_b });
wire [7:0] wd_i = preIndex ? insn[31:24] : insn[15:8];
wire [7:0] wd8 = i[LDMN] ? wd_i : wd_r8;
assign wd_hl = preIndex ? indexSw ? fwd_iy : fwd_ix : { fwd_h, fwd_l };
wire [15:0] wd_spaf = i[PUSH] ? { fwd_a, fwd_f } : sp;
wire [15:0] wd_r16 = sel4x16(o[5:4],
	{ wd_spaf, wd_hl, fwd_d, fwd_e, fwd_b, fwd_c });
wire [15:0] wd16 = i[CALL] | i[RST] | nmiack ?
	(intack | nmiack) & ~halt ? pc : nextpc_normal : wd_r16;

wire mw_s0_u = ~state & (i[LD_NNHL] | i[LD_NNSS] | i[PUSH] | exec_call | i[RST] | nmiack);
wire mw_s0_l = ~state & (i[LDMR] | i[LDMN] | i[LD_BCDENNA]);
reg iord;
always @(posedge clk)
	if (reset) iord <= 1'b0;
	else if (clken) iord <= ~state & (i[INAN] | i[INRC] | i[IOX] & ~s[0]);
wire iowr_s0 = ~state & (i[OUTNA] | i[OUTCR]);
wire [15:0] wd_s0 = { wd16[15:8], mw_s0_u ? wd16[7:0] : wd8 };

//
// EXEC
//

reg [31:0] insn1;
reg [7:0] o1;
reg [I1MAX:0] i1;
reg preCB1, preIndex1, indexSw1, dbl_state1;
always @(posedge clk) if (clken) begin
	insn1 <= insn;
	i1 <= i[I1MAX:0];
	o1 <= o;
	preCB1 <= preCB;
	preIndex1 <= preIndex;
	indexSw1 <= indexSw;
	dbl_state1 <= dbl_state;
end

// register input selector (8bit)

wire t_data = i[LDA_BCDENN] | i[LDHL_NN] | i[INAN] | i[INRC] | i[BLOCK];
wire [2:0] t_sel_alu_r = i[INCDEC8] | i[LAL8] | preCB ?
	i[INCDEC8] ? o[5:3] : o[2:0] :
	{ 2'b11, ~t_data };
reg [2:0] sel_alu_r;
always @(posedge clk)
	if (clken) sel_alu_r <= t_sel_alu_r & {3{ ~(i[DJNZ] | i[IOX]) }};
wire [15:0] hl_alu = preIndex1 ? indexSw1 ? iy : ix : { h, l };
wire [7:0] alu_r = sel8x8(sel_alu_r,
	{ a, data[7:0], hl_alu[7:0], hl_alu[15:8], e, d, c, b });

// register input selector (16bit)

wire sel_by54 = i[LDSSNN] | i[LDSS_NN] | i[LD_NNSS] | i[PUSHPOP] |
	i[ADD16] | i[ADCSBC16] | i[INCDEC16];
reg [1:0] sel16;
always @(posedge clk)
	if (clken) sel16 <= { ~sel_by54 | o[5], sel_by54 & o[4] } & {2{ ~(i[LDX] | i[CPX]) }};
wire [15:0] spaf = i1[PUSHPOP] ? { a, f } : sp;
wire [15:0] alu16_b = sel4x16(sel16, { spaf, hl_alu, d, e, b, c });

// imm. input selector

// imm2: ld|add|adc|sub|sbc|and|or|xor|cp r,n
// imm3: ld ixl|ixh|iyl|iyh,n
// imm4: ld (ix|iy),n
wire imm34 = ~|s[7:6] & s[2:0] == 3'b110 & preIndex;
wire imm2 = ~^p[7:6] & p[2:0] == 3'b110;
wire imm4 = imm34 & s[5:3] == 3'b110;
reg [1:0] sel_i_sel;
always @(posedge clk)
	if (clken) sel_i_sel <= { imm34, imm4 | imm2 };
wire daa_l = f[4] | a[3:0] >= 4'b1010;
wire daa_u = f[0] | (a[7:4] >= 4'b1010 | a[7:4] == 4'b1001 & a[3:0] >= 4'b1010);
wire [7:0] daa_adj = { 1'b0, daa_u, daa_u, 2'b00, daa_l, daa_l, 1'b0 };
wire [7:0] sel_i = sel4x8(sel_i_sel, { insn1[31:8], daa_adj });
wire [7:0] alu_b = |sel_i_sel ? sel_i : alu_r;

// arith (8bit)

reg t_isadd8, add_a_valid, add_a_ff, withCy8, cy8_1;
always @(posedge clk) if (clken) begin
	t_isadd8 <= i[INCDEC8] | i[DJNZ] | i[IOX] | i[ADDADC8];
	add_a_valid <= ~(i[CPL] | i[NEG] | i[INCDEC8] | i[DJNZ] | i[IOX]);
	add_a_ff <= i[INCDEC8] & o[0] | i[DJNZ] | i[IOX];
	withCy8 <= i[ARITH8] & o[3];
	cy8_1 <= i[NEG] | i[INCDEC8] & ~o[0];
end
wire isadd8 = t_isadd8 | i1[DAA] & ~f[1];
wire [7:0] add_a = a & {8{ add_a_valid }} | {8{ add_a_ff }};
wire [7:0] add_b = alu_b ~^ {8{ isadd8 }};
wire add_c = (withCy8 & f[0] ~^ isadd8) & ~i1[CPL] | cy8_1;
wire [8:0] add_y = add_a + add_b + add_c;

// arith (16bit)

reg add16_a_valid, issub16, add16_a_ffff, cy16_1;
always @(posedge clk) if (clken) begin
	add16_a_valid <= i[ADD16] | i[ADCSBC16];
	issub16 <= i[ADCSBC16] & ~o[3];
	add16_a_ffff <= i[INCDEC16] & o[3] | i[LDX] | i[CPX];
	cy16_1 <= i[INCDEC16] & ~o[3];
end
wire [15:0] add16_a = hl_alu & {16{ add16_a_valid }} | {16{ add16_a_ffff }};
wire [15:0] add16_b = alu16_b ^ {16{ issub16 }};
wire add16_c = (i1[ADCSBC16] & f[0]) ^ issub16 | cy16_1;
wire [16:0] add16_y = add16_a + add16_b + add16_c;

// logic

wire l_and = i[AND] | ^o[7:6] & preCB;
wire l_or = i[OR] | &o[7:6] & preCB;
reg [1:0] sel_logic;
reg [7:0] logic_m;
always @(posedge clk) if (clken) begin
	sel_logic <= { i[XOR] | l_or , i[XOR] | l_and };
	logic_m <= 8'h01 << o[5:3] ^ {8{o[7:6] == 2'b10}};
end
wire [7:0] logic_a = preCB1 ? logic_m : a;
wire [7:0] logic_y = sel4x8(sel_logic,
	{ logic_a ^ alu_b, logic_a | alu_b, logic_a & alu_b, alu_b });

// ALU sel (8bit)

reg t_arith, t_left;
reg [1:0] sel_sftrot;
always @(posedge clk) if (clken) begin
	t_arith <= i[ARITH8CP] | i[INCDEC8] | i[DAA] | i[NEG] |
		i[CPL] | i[CPX] | i[DJNZ] | i[IOX];
	t_left <= i[SRALL] & ~o[3];
	sel_sftrot <= { preCB & o[5], o[4] };
end
wire [3:0] left0_a = { 2'b10, f[0], alu_r[7] };
wire [3:0] right7_a = { 1'b0, alu_r[7], f[0], alu_r[0] };
wire [7:0] alu_y = i1[SRALL] ?
	t_left ? { alu_r[6:0], left0_a[sel_sftrot] } :
		{ right7_a[sel_sftrot], alu_r[7:1] } :
	t_arith ? add_y[7:0] : logic_y;
assign alu_y_is0 = ~|alu_y;

// ALU sel (16bit)

reg sel_data16;
reg [15:0] imm16;
always @(posedge clk) if (clken) begin
	sel_data16 <= i[LDHL_NN] | i[LDSS_NN] | i[PUSHPOP] | i[EXSPHL];
	imm16 <= imm16_s;
end
wire [15:0] alu16_y = i1[LDSSNN] ? imm16 : sel_data16 ? data : add16_y[15:0];
wire alu16_y_is0 = ~|alu16_y;

// nibble selector

wire [3:0] rldrrd_au = a[7:4];
wire [3:0] rldrrd_al = o1[3] ? data[7:4] : data[3:0];
wire [3:0] rldrrd_mu = o1[3] ? data[3:0] : a[3:0];
wire [3:0] rldrrd_ml = o1[3] ? a[3:0] : data[7:4];
wire [7:0] wd_a = i1[RLDRRD] ? { rldrrd_au, rldrrd_al } : alu_y;

// write data (after read)

reg sel_alu16, t_mw_s1_l;
always @(posedge clk) if (clken) begin
	sel_alu16 <= i[BLOCK] |
		i[LDHL_NN] | i[LD_NNHL] | i[LDSSNN] | i[LDSS_NN] | i[LDSPHL] |
		i[EXSPHL] | i[PUSHPOP] | i[ADD16] | i[ADCSBC16] | i[INCDEC16];
	t_mw_s1_l <= dbl_state & ~i[IOX];
end
wire mw_s1_l = state & t_mw_s1_l;
wire mw_s1_u = state & i1[EXSPHL];
wire iowr_s1 = state & i1[IOX] & o1[0];
wire [7:0] wd_s1_l = i1[RLDRRD] ? { rldrrd_mu, rldrrd_ml } :
	sel_alu16 ? alu16_y[7:0] : alu_y;
wire [15:0] wd_s1 = { alu16_y[15:8], wd_s1_l };
assign mw_u = ~busack & (mw_s0_u | mw_s1_u);
assign mw_l = ~busack & (mw_s0_l | mw_s1_l | mw_s0_u | mw_s1_u);
assign iowr = iowr_s0 | iowr_s1;
assign data_out = i1[LDX] & state ? data :
	i1[EXSPHL] & state ? hl_alu : mw_s1_l | mw_s1_u ? wd_s1 : wd_s0;

//
// UPDATE
//

wire load8 = i[LDRR] | i[LDRN] | i[INCDEC8] | i[INRC] | i[EXCBIT];
wire [2:0] sel_load = preCB ? o[2:0] : o[5:3];
wire t_load16 = i[LDSSNN] | i[LDSS_NN] | i[INCDEC16];
wire load16 = t_load16 | i[POP];
wire loadaf = i[POP] & &o[5:4];
wire load_hl = i[LDHL_NN] | i[ADD16] | i[ADCSBC16] | i[EXSPHL] & ~state;
wire l_index = preIndex & ~(i[LDRM] & o[5:4] == 2'b10);// ld r,(ix|iy);r!=l,r!=h
wire match01 = match | match1;
reg load16_1, loadaf1, load16_hl1, l_index1, fsw_hl1, fsw_ix1, fsw_iy1, sp_sub1;
always @(posedge clk) if (clken) begin
	load16_1 <= load16;
	loadaf1 <= loadaf;
	load16_hl1 <= load16 | load_hl;
	l_index1 <= l_index;
	fsw_hl1 <= ~i[BLOCK] & ~l_index;
	fsw_ix1 <= l_index & ~indexSw;
	fsw_iy1 <= l_index &  indexSw;
	sp_sub1 <= o[2] & ~preED | nmiack;
end

wire [15:0] de_d = { d, e } + { {15{ o1[3] }}, 1'b1 };
wire [15:0] hl_d = { h, l } + { {15{ o1[3] }}, 1'b1 };
wire [15:0] sp_d = { sp[15:1] + { {14{ sp_sub1 }}, 1'b1 }, sp[0] };

wire ren = clken & (~dbl_state1 | state);

// A register

reg load_a1;
always @(posedge clk)
	if (clken) load_a1 <= i[LDA_BCDENN] | i[LDAIR] | i[ARITH8] | i[DAA] |
		i[LOGIC] | i[RA] | i[CPL] | i[INAN] |
		i[RLDRRD] & ~state |
		load8 & &sel_load |
		loadaf;
wire [7:0] r_or_i = o1[3] ? rr : ri;
assign fwd_a = load_a1 ?
	i1[LDAIR] ? r_or_i : load16_1 ? alu16_y[15:8] : wd_a : a;
always @(posedge clk)
	if (reset) begin
		ra[0] <= 8'hff;
		ra[1] <= 8'hff;
	end
	else if (ren & load_a1)
		ra[sel_af] <= fwd_a;

// B register

reg t_load_b1;
always @(posedge clk)
	if (clken) t_load_b1 <= i[DJNZ] | i[LDX] | i[CPX] & ~exit16 |
		load8 & sel_load == 3'b000 |
		load16 & o[5:4] == 2'b00;
wire load_b1 = t_load_b1 & ~match01;
assign fwd_b = load_b1 & ~i1[BLOCK] ? load16_1 ? alu16_y[15:8] : alu_y : b;
always @(posedge clk)
	if (reset) begin
		rb[0] <= 8'h00;
		rb[1] <= 8'h00;
	end
	else if (ren & load_b1)
		rb[sel_bcdehl] <= i1[BLOCK] ? alu16_y[15:8] : fwd_b;

// C register

reg t_load_c1;
always @(posedge clk)
	if (clken) t_load_c1 <= i[LDX] | i[CPX] & ~exit16 |
		load8 & sel_load == 3'b001 |
		load16 & o[5:4] == 2'b00;
wire load_c1 = t_load_c1 & ~match01;
assign fwd_c = load_c1 & ~i1[BLOCK] ? load16_1 ? alu16_y[7:0] : alu_y : c;
always @(posedge clk)
	if (reset) begin
		rc[0] <= 8'h00;
		rc[1] <= 8'h00;
	end
	else if (ren & load_c1)
		rc[sel_bcdehl] <= i1[BLOCK] ? alu16_y[7:0] : fwd_c;

// D register

reg load_d1;
always @(posedge clk)
	if (clken) load_d1 <= i[EXDEHL] | i[LDX] |
		load8 & sel_load == 3'b010 |
		load16 & o[5:4] == 2'b01;
assign fwd_d = load_d1 & ~i1[BLOCK] ?
	i1[EXDEHL] ? h : load16_1 ? alu16_y[15:8] : alu_y : d;
always @(posedge clk)
	if (reset) begin
		rd[0] <= 8'h00;
		rd[1] <= 8'h00;
	end
	else if (ren & load_d1)
		rd[sel_bcdehl] <= i1[BLOCK] ? de_d[15:8] : fwd_d;

// E register

reg load_e1;
always @(posedge clk)
	if (clken) load_e1 <= i[EXDEHL] | i[LDX] |
		load8 & sel_load == 3'b011 |
		load16 & o[5:4] == 2'b01;
assign fwd_e = load_e1 & ~i1[BLOCK] ?
	i1[EXDEHL] ? l : load16_1 ? alu16_y[7:0] : alu_y : e;
always @(posedge clk)
	if (reset) begin
		re[0] <= 8'h00;
		re[1] <= 8'h00;
	end
	else if (ren & load_e1)
		re[sel_bcdehl] <= i1[BLOCK] ? de_d[7:0] : fwd_e;

// H register

reg t_load_h1;
always @(posedge clk)
	if (clken) t_load_h1 <= i[EXDEHL] | i[LDX] | i[CPX] & ~exit16 |
		load_hl |
		load8 & sel_load == 3'b100 |
		load16 & o[5:4] == 2'b10;
wire [7:0] fwd_ht = i1[EXDEHL] ? d : load16_hl1 ? alu16_y[15:8] : alu_y;
assign fwd_h = t_load_h1 & ~match01 & fsw_hl1 ? fwd_ht : h;
assign fwd_ixh = t_load_h1 & fsw_ix1 ? fwd_ht : ixh;
assign fwd_iyh = t_load_h1 & fsw_iy1 ? fwd_ht : iyh;
always @(posedge clk)
	if (reset) begin
		rh[0] <= 8'h00;
		rh[1] <= 8'h00;
		ixh <= 8'h00;
		iyh <= 8'h00;
	end
	else if (ren)
		if (l_index1) begin
			if (t_load_h1)
				if (indexSw1) iyh <= fwd_iyh;
				else ixh <= fwd_ixh;
		end
		else if (t_load_h1 & ~match01)
			rh[sel_bcdehl] <= i1[BLOCK] ? hl_d[15:8] : fwd_h;

// L register

reg t_load_l1;
always @(posedge clk)
	if (clken) t_load_l1 <= i[EXDEHL] | i[LDX] | i[CPX] & ~exit16 |
		load_hl |
		load8 & sel_load == 3'b101 |
		load16 & o[5:4] == 2'b10;
wire [7:0] fwd_lt = i1[EXDEHL] ? e : load16_hl1 ? alu16_y[7:0] : alu_y;
assign fwd_l = t_load_l1 & ~match01 & fsw_hl1 ? fwd_lt : l;
assign fwd_ixl = t_load_l1 & fsw_ix1 ? fwd_lt : ixl;
assign fwd_iyl = t_load_l1 & fsw_iy1 ? fwd_lt : iyl;
always @(posedge clk)
	if (reset) begin
		rl[0] <= 8'h00;
		rl[1] <= 8'h00;
		ixl <= 8'h00;
		iyl <= 8'h00;
	end
	else if (ren)
		if (l_index1) begin
			if (t_load_l1)
				if (indexSw1) iyl <= fwd_iyl;
				else ixl <= fwd_ixl;
		end
		else if (t_load_l1 & ~match01)
			rl[sel_bcdehl] <= i1[BLOCK] ? hl_d[7:0] : fwd_l;

// SP register

wire sp_addsub = exec_call | i[RST] | exec_ret | i[PUSHPOP] | nmiack;
reg load_sp1, sp_addsub1;
always @(posedge clk) if (clken) begin
	load_sp1 <= t_load16 & &o[5:4] | i[LDSPHL] | sp_addsub;
	sp_addsub1 <= sp_addsub;
end
assign fwd_sp = load_sp1 ? sp_addsub1 ? sp_d : alu16_y : sp;
always @(posedge clk)
	if (reset)
		sp <= 16'hffff;
	else if (ren & load_sp1)
		sp <= fwd_sp;

// F register

reg f_n1, f_h1, f_h16_1, f_ho1, f_p1, f_zs1;
always @(posedge clk) if (clken) begin
	f_n1 <= i[CPL] | i[NEG] | i[IOX];
	f_h1 <= i[ARITH8CP] | i[DAA];
	f_h16_1 <= i[ADD16] | i[ADCSBC16];
	f_ho1 <= i[AND] | i[CPL] | i[BIT];
	f_p1 <= i[LOGIC] | i[SRCB] | i[DAA] | i[INRC];
	f_zs1 <= i[ARITH8CP] | i[INCDEC8] | i[DAA] | i[LOGIC] | i[SRCB] | i[CPX] | i[INRC];
end

wire c3add = add_a[3] & alu_b[3] | alu_b[3] & ~add_y[3] | ~add_y[3] & add_a[3];
wire c3sub = ~add_a[3] & alu_b[3] | alu_b[3] & add_y[3] | add_y[3] & ~add_a[3];
wire v7 = add_a[7] & add_b[7] & ~add_y[7] | ~add_a[7] & ~add_b[7] & add_y[7];
wire c11add = add16_a[11] & alu16_b[11] | alu16_b[11] & ~add16_y[11] | ~add16_y[11] & add16_a[11];
wire c11sub = ~add16_a[11] & alu16_b[11] | alu16_b[11] & add16_y[11] | add16_y[11] & ~add16_a[11];
wire v15 = add16_a[15] & add16_b[15] & ~add16_y[15] | ~add16_a[15] & ~add16_b[15] & add16_y[15];

wire t_c = (i1[ARITH8CP] | i1[DAA]) & (isadd8 ~^ add_y[8]) |
	i1[ADCSBC16] & (issub16 ^ add16_y[16]) |
	i1[ADD16] & add16_y[16] |
	i1[SRALL] & (o1[3] ? alu_r[0] : alu_r[7]) |
	i1[CCF] & ~f[0] |
	i1[SCF];

wire t_n = i1[ARITH8CP] & ~isadd8 |
	i1[INCDEC8] & o1[0] |
	i1[ADCSBC16] & issub16 |
	f_n1;

wire t_p = (i1[ARITH8CP] | i1[INCDEC8]) & v7 |
	i1[ADCSBC16] & v15 |
	f_p1 & ~^alu_y |
	i1[BIT] & alu_y_is0 |
	(i1[LDX] | i1[CPX]) & ~exit16 | i1[LDAIR] & iff2;

wire t_h = f_h1 & (t_n ? c3sub : c3add) |
	f_h16_1 & (t_n ? c11sub : c11add) |
	i1[INCDEC8] & (t_n ? add_y[3] & ~alu_b[3] : ~add_y[3] & alu_b[3]) |
	f_ho1;

wire t_z = (f_zs1 | i1[BIT]) & alu_y_is0 |
	i1[ADCSBC16] & alu16_y_is0 |
	i1[IOX] & exit8 |
	i1[LDAIR] & ~|r_or_i;

wire t_s = f_zs1 & alu_y[7] |
	i1[ADCSBC16] & add16_y[15] |
	i1[LDAIR] & r_or_i[7];

wire f_all = i[ARITH8CP] | i[ADCSBC16] | i[LOGIC] | i[SRCB] & ~state;
wire f_szhpn = i[INCDEC8] & ~state | i[INRC] | i[CPX] | i[LDAIR];
reg update_c, update_n, update_p, update_h, update_z, update_s;
always @(posedge clk) if (clken) begin
	update_c <= f_all | i[ADD16] | i[RA] | i[DAA] | i[CCF] | i[SCF];
	update_n <= f_all | f_szhpn | i[ADD16] | i[RA] | i[CPL] | i[CCF] | i[SCF] | i[BIT] | (i[LDX] | i[IOX]) & ~state;
	update_p <= f_all | f_szhpn | i[DAA] | i[BIT] | i[LDX] & ~state;
	update_h <= f_all | f_szhpn | i[ADD16] | i[RA] | i[DAA] | i[CPL] | i[SCF] | i[LDX] & ~state;
	update_z <= f_all | f_szhpn | i[DAA] | i[BIT] | i[IOX] & ~state;
	update_s <= f_all | f_szhpn | i[DAA];
end

// not implemented: X,Y
assign fwd_f = loadaf1 ? alu16_y[7:0] : {
	update_s ? t_s : f[7],
	update_z ? t_z : f[6],
	f[5],
	update_h ? t_h : f[4],
	f[3],
	update_p ? t_p : f[2],
	update_n ? t_n : f[1],
	update_c ? t_c : f[0]
};
always @(posedge clk)
	if (reset) begin
		rf[0] <= 8'hff;
		rf[1] <= 8'hff;
	end
	else if (ren & ~match1) rf[sel_af] <= fwd_f;

// I register

always @(posedge clk)
	if (ren & i1[LDIA]) ri <= a;

// R register

wire ren_r = clken & active & ~force_nop & (~dbl_state | state);
wire [6:0] t_r = rr[6:0] + { 5'b00000, ~nopre, nopre };
wire [7:0] fwd_r = { rr[7], t_r };
always @(posedge clk)
	if (reset) rr <= 0;
	else if (ren_r)
		if (i1[LDRA]) rr <= a;
		else if (~(i[CPX] & ~i1[CPX])) rr[6:0] <= t_r;

// intmode

always @(posedge clk)
	if (reset) intmode <= 2'b00;
	else if (ren & i1[IM]) intmode <= o1[4:3];

// INT/NMI

reg nmireq1;
always @(posedge clk)
	if (reset) nmireq1 <= 1'b0;
	else nmireq1 <= nmireq;

reg nmiedge;
always @(posedge clk)
	if (reset) nmiedge <= 1'b0;
	else if (nmireq & ~nmireq1) nmiedge <= 1'b1;
	else if (nmiedge & nmiack) nmiedge <= 1'b0;

always @(posedge clk)
	if (reset) begin
		iff1 <= 1'b0;
		iff2 <= 1'b0;
	end
	else if (nmiedge & accept) begin
		iff1 <= 1'b0;
		if (i1[EIDI]) iff2 <= o1[3];
	end
	else if (intreq & iff1 & accept) begin
		iff1 <= 1'b0;
		iff2 <= 1'b0;
	end
	else if (i1[EIDI]) begin
		iff1 <= o1[3];
		iff2 <= o1[3];
	end
	else if (i1[RETIN]) iff1 <= iff2;

always @(posedge clk)
	if (reset) nmiack <= 1'b0;
	else if (nmiack) nmiack <= 1'b0;
	else if (nmiedge & accept) nmiack <= 1'b1;

always @(posedge clk)
	if (reset) intack <= 1'b0;
	else if (intack) intack <= 1'b0;
	else if (intreq & iff1 & accept) intack <= 1'b1;

wire [15:0] bc = { fwd_b, fwd_c }, de = { fwd_d, fwd_e }, hl = { fwd_h, fwd_l }, af = { fwd_a, fwd_f };
initial $monitor("%04x %x %02x %04x %04x %04x %02x %02x  %x%xM %04x %04x %04x",
	pc, force_nop, o, bc, de, hl, fwd_a, fwd_f,  mw_u, mw_l, adr_out, data_out, data_in);
endmodule
