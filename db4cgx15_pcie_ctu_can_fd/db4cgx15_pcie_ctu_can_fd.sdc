create_clock -name {clk25b} -period 40 -waveform { 0.000 20.000 } [get_ports clk25b]
create_clock -name {pcie_ref_clk} -period 10.000 -waveform { 0.000 5.000 } [get_ports {pcie_ref_clk}]

# CAN RX line synchroniser attirbutes
set CAN_rx_chain_1 [get_cells {*bus_sync_comp/sync_Chain_1*}]
set CAN_rx_chain_2 [get_cells {*bus_sync_comp/sync_Chain_2*}]

set_property ASYNC_REG true $CAN_rx_chain_1
set_property ASYNC_REG true $CAN_rx_chain_2
