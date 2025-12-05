create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}] -add
create_generated_clock -name serial_clk -source [get_ports {clk}] -master_clock clk -divide_by 8 -multiply_by 27 [get_nets {serial_clk}]
create_generated_clock -name video_clk -source [get_nets {serial_clk}] -master_clock serial_clk -divide_by 5 [get_nets {video_clk}]
//create_generated_clock -name cpu_clk -source [get_ports {clk}] -master_clock clk -divide_by 64 [get_nets {cpu_clk}]
set_clock_groups -asynchronous
	-group [get_clocks {clk}] 
	-group [get_clocks {serial_clk}]
	-group [get_clocks {video_clk}]
//	-group [get_clocks {cpu_clk}]
