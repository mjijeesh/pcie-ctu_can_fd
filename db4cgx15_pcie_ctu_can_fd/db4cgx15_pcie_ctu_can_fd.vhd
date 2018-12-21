-- Copyright (C) 2017  Intel Corporation. All rights reserved.
-- Your use of Intel Corporation's design tools, logic functions 
-- and other software and tools, and its AMPP partner logic 
-- functions, and any output files from any of the foregoing 
-- (including device programming or simulation files), and any 
-- associated documentation or information are expressly subject 
-- to the terms and conditions of the Intel Program License 
-- Subscription Agreement, the Intel Quartus Prime License Agreement,
-- the Intel MegaCore Function License Agreement, or other 
-- applicable license agreement, including, without limitation, 
-- that your use is for the sole purpose of programming logic 
-- devices manufactured by Intel and sold by Intel or its 
-- authorized distributors.  Please refer to the applicable 
-- agreement for further details.

library ieee;
use ieee.std_logic_1164.all;
library altera;
use altera.altera_syn_attributes.all;

entity db4cgx15_pcie_ctu_can_fd is
	port
	(
-- {ALTERA_IO_BEGIN} DO NOT REMOVE THIS LINE!

-- {ALTERA_IO_END} DO NOT REMOVE THIS LINE!
        clk25b  : in  std_logic;

        pcie_rx0 : in  std_logic;
        pcie_tx0 : out std_logic;
        pcie_ref_clk : in  std_logic;
        pcie_reset_n : in  std_logic;
        reset_n : in  std_logic;
        pcie_wake : in  std_logic
	);

-- {ALTERA_ATTRIBUTE_BEGIN} DO NOT REMOVE THIS LINE!
-- {ALTERA_ATTRIBUTE_END} DO NOT REMOVE THIS LINE!
end db4cgx15_pcie_ctu_can_fd;

architecture ppl_type of db4cgx15_pcie_ctu_can_fd is

	component pcie_core is
		port (
			clk_25_clk                                              : in  std_logic                     := 'X';             -- clk
			reset_n_reset_n                                         : in  std_logic                     := 'X';             -- reset_n

			pcie_hard_ip_0_pcie_rstn_export                         : in  std_logic                     := 'X';             -- export
			pcie_hard_ip_0_reconfig_busy_busy_altgxb_reconfig       : in  std_logic                     := 'X';             -- busy_altgxb_reconfig
			pcie_hard_ip_0_refclk_export                            : in  std_logic                     := 'X';             -- export
			pcie_hard_ip_0_rx_in_rx_datain_0                        : in  std_logic                     := 'X';             -- rx_datain_0
			pcie_hard_ip_0_tx_out_tx_dataout_0                      : out std_logic;                                        -- tx_dataout_0

			external_bus_interface_acknowledge : in  std_logic                     := 'X';             -- acknowledge
			external_bus_interface_irq         : in  std_logic                     := 'X';             -- irq
			external_bus_interface_address     : out std_logic_vector(15 downto 0);                    -- address
			external_bus_interface_bus_enable  : out std_logic;                                        -- bus_enable
			external_bus_interface_byte_enable : out std_logic_vector(3 downto 0);                     -- byte_enable
			external_bus_interface_rw          : out std_logic;                                        -- rw
			external_bus_interface_write_data  : out std_logic_vector(31 downto 0);                    -- write_data
			external_bus_interface_read_data   : in  std_logic_vector(31 downto 0) := (others => 'X'); -- read_data

			pcie_hard_ip_0_clocks_sim_clk250_export                 : out std_logic;                                        -- clk250_export
			pcie_hard_ip_0_clocks_sim_clk500_export                 : out std_logic;                                        -- clk500_export
			pcie_hard_ip_0_clocks_sim_clk125_export                 : out std_logic;                                        -- clk125_export
			pcie_hard_ip_0_pipe_ext_pipe_mode                       : in  std_logic                     := 'X';             -- pipe_mode
			pcie_hard_ip_0_pipe_ext_phystatus_ext                   : in  std_logic                     := 'X';             -- phystatus_ext
			pcie_hard_ip_0_pipe_ext_rate_ext                        : out std_logic;                                        -- rate_ext
			pcie_hard_ip_0_pipe_ext_powerdown_ext                   : out std_logic_vector(1 downto 0);                     -- powerdown_ext
			pcie_hard_ip_0_pipe_ext_txdetectrx_ext                  : out std_logic;                                        -- txdetectrx_ext
			pcie_hard_ip_0_pipe_ext_rxelecidle0_ext                 : in  std_logic                     := 'X';             -- rxelecidle0_ext
			pcie_hard_ip_0_pipe_ext_rxdata0_ext                     : in  std_logic_vector(7 downto 0)  := (others => 'X'); -- rxdata0_ext
			pcie_hard_ip_0_pipe_ext_rxstatus0_ext                   : in  std_logic_vector(2 downto 0)  := (others => 'X'); -- rxstatus0_ext
			pcie_hard_ip_0_pipe_ext_rxvalid0_ext                    : in  std_logic                     := 'X';             -- rxvalid0_ext
			pcie_hard_ip_0_pipe_ext_rxdatak0_ext                    : in  std_logic                     := 'X';             -- rxdatak0_ext
			pcie_hard_ip_0_pipe_ext_txdata0_ext                     : out std_logic_vector(7 downto 0);                     -- txdata0_ext
			pcie_hard_ip_0_pipe_ext_txdatak0_ext                    : out std_logic;                                        -- txdatak0_ext
			pcie_hard_ip_0_pipe_ext_rxpolarity0_ext                 : out std_logic;                                        -- rxpolarity0_ext
			pcie_hard_ip_0_pipe_ext_txcompl0_ext                    : out std_logic;                                        -- txcompl0_ext
			pcie_hard_ip_0_pipe_ext_txelecidle0_ext                 : out std_logic;                                        -- txelecidle0_ext
			pcie_hard_ip_0_powerdown_pll_powerdown                  : in  std_logic                     := 'X';             -- pll_powerdown
			pcie_hard_ip_0_powerdown_gxb_powerdown                  : in  std_logic                     := 'X';             -- gxb_powerdown
			pcie_hard_ip_0_test_in_test_in                          : in  std_logic_vector(39 downto 0) := (others => 'X'); -- test_in
			pcie_hard_ip_0_test_out_test_out                        : out std_logic_vector(8 downto 0);                     -- test_out

         clk_50_clk                                              : out std_logic;
         pcie_core_clk_clk                                       : out std_logic 
	);
	end component pcie_core;


-- {ALTERA_COMPONENTS_BEGIN} DO NOT REMOVE THIS LINE!
-- {ALTERA_COMPONENTS_END} DO NOT REMOVE THIS LINE!
begin
-- {ALTERA_INSTANTIATION_BEGIN} DO NOT REMOVE THIS LINE!
-- {ALTERA_INSTANTIATION_END} DO NOT REMOVE THIS LINE!

	pcie_core_inst : component pcie_core
		port map (
			clk_25_clk                                              => clk25b,			-- clk_25.clk
			reset_n_reset_n                                         => reset_n,       --  reset.reset_n
			
			pcie_hard_ip_0_refclk_export                            => pcie_ref_clk,   -- pcie_hard_ip_0_refclk.export
			pcie_hard_ip_0_rx_in_rx_datain_0                        => pcie_rx0,       -- pcie_hard_ip_0_rx_in.rx_datain_0
			pcie_hard_ip_0_tx_out_tx_dataout_0                      => pcie_tx0,       -- pcie_hard_ip_0_tx_out.tx_dataout_0
			pcie_hard_ip_0_pcie_rstn_export                         => pcie_reset_n,   -- pcie_hard_ip_0_pcie_rstn.export

			pcie_hard_ip_0_reconfig_busy_busy_altgxb_reconfig       => open,          --  pcie_hard_ip_0_reconfig_busy.busy_altgxb_reconfig
			external_bus_interface_acknowledge => '1',           -- external_bus_interface.acknowledge
			external_bus_interface_irq         => '0',           -- .irq
			external_bus_interface_address     => open,          -- .address
			external_bus_interface_bus_enable  => open,          -- .bus_enable
			external_bus_interface_byte_enable => open,          -- .byte_enable
			external_bus_interface_rw          => open,          -- .rw
			external_bus_interface_write_data  => open,          -- .write_data
			external_bus_interface_read_data   => x"12345678",   -- .read_data

			clk_50_clk                         => open,
			pcie_core_clk_clk                  => open
		);

		
end;
