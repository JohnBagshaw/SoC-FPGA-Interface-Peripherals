# SoC-FPGA-Interface-Peripherals
SoC FPGA interface HDL codes
SPI interface code in Verilog HDL for a 12-bit ADC using Quartus Prime, ModelSim and SignalTap for Altera's Cyclone IV DE0-Nano Eval board and can easily be modified for other SPI interface specifications also using Xilinx Vivado/Vitis
For all interface peripherals, start with reveiwing the timing diagrams.

# Design Workflow
- Create BDF
- Use PLL to set clk
- Add I/O pins
- Perform pin assignment using the pin planner
- Compile design
- Perform analysis and synthesis
- Perform real-time debugging with SignalTap logic analyzer
- Program device
- Construct the expected 12-bit word
