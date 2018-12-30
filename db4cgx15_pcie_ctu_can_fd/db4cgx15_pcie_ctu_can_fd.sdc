create_clock -name {clk25b} -period 40 -waveform { 0.000 20.000 } [get_ports clk25b]
create_clock -name {pcie_ref_clk} -period 10.000 -waveform { 0.000 5.000 } [get_ports {pcie_ref_clk}]
