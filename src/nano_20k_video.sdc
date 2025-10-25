//Copyright (C)2014-2025 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.12 
//Created Time: 2025-10-24 14:31:07
create_clock -name tmds_clk -period 29.63 -waveform {0 14.815} [get_pins {u_clkdiv/CLKOUT}]
create_clock -name I_clk -period 37.037 -waveform {0 18.518} [get_ports {I_clk}] -add
