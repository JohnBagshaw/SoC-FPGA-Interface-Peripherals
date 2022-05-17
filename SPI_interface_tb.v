'timescale 10ns/100ps

module SPI_interface_tb;

reg clock;
reg dout_s;
wire sclk_s;
wire din_s;
wire [3:0] count_s;

initial
begin
clock = 0;
end

always
begin
#10 clock = !clock;
end

SPI_interface SPI_interface_tb_inst
(
	.sclk(sclk_s), //output sclk_s
	.clk(clock), // input clk_s
	.din(din_s), // input din_s
	.chipsel(chipsel_s), // output chipsel_s
	.dout(dout_s), // output dout_s
	.count(count_s)
)
