--------------------------------------------------------------------------------
-- 
-- CTU CAN FD IP Core
-- Copyright (C) 2015-2018
-- 
-- Authors:
--     Ondrej Ille <ondrej.ille@gmail.com>
--     Martin Jerabek <martin.jerabek01@gmail.com>
--     Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 
-- Project advisors: 
-- 	Jiri Novak <jnovak@fel.cvut.cz>
-- 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
-- 
-- Department of Measurement         (http://meas.fel.cvut.cz/)
-- Faculty of Electrical Engineering (http://www.fel.cvut.cz)
-- Czech Technical University        (http://www.cvut.cz/)
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this VHDL component and associated documentation files (the "Component"),
-- to deal in the Component without restriction, including without limitation
-- the rights to use, copy, modify, merge, publish, distribute, sublicense,
-- and/or sell copies of the Component, and to permit persons to whom the
-- Component is furnished to do so, subject to the following conditions:
-- 
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Component.
-- 
-- THE COMPONENT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
-- FROM, OUT OF OR IN CONNECTION WITH THE COMPONENT OR THE USE OR OTHER DEALINGS
-- IN THE COMPONENT.
-- 
-- The CAN protocol is developed by Robert Bosch GmbH and protected by patents.
-- Anybody who wants to implement this IP core on silicon has to obtain a CAN
-- protocol license from Bosch.
-- 
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.can_constants.all;
use work.can_components.all;

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
        pcie_wake : in  std_logic;

        can_rx : in std_logic;
        can_tx : out std_logic

	);

-- {ALTERA_ATTRIBUTE_BEGIN} DO NOT REMOVE THIS LINE!
-- {ALTERA_ATTRIBUTE_END} DO NOT REMOVE THIS LINE!
end db4cgx15_pcie_ctu_can_fd;

architecture ppl_type of db4cgx15_pcie_ctu_can_fd is

   constant can_fd_instances_number : natural := 2;
   signal can_fd_local_feedback : std_logic := '0';

	component pcie_core is
		port (
			clk_25_clk                                              : in  std_logic                     := 'X';             -- clk
			reset_n_reset_n                                         : in  std_logic                     := 'X';             -- reset_n

			pcie_hard_ip_0_pcie_rstn_export                         : in  std_logic                     := 'X';             -- export
			pcie_hard_ip_0_refclk_export                            : in  std_logic                     := 'X';             -- export
			pcie_hard_ip_0_rx_in_rx_datain_0                        : in  std_logic                     := 'X';             -- rx_datain_0
			pcie_hard_ip_0_tx_out_tx_dataout_0                      : out std_logic;                                        -- tx_dataout_0

			pcie_hard_ip_0_reconfig_busy_busy_altgxb_reconfig       : in  std_logic                     := 'X';             -- busy_altgxb_reconfig
			pcie_hard_ip_0_reconfig_fromgxb_0_data                  : out std_logic_vector(4 downto 0);                     -- data
			pcie_hard_ip_0_reconfig_togxb_data                      : in  std_logic_vector(3 downto 0)  := (others => 'X'); -- data

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
         clk_100_clk                                             : out std_logic;
         pcie_core_clk_clk                                       : out std_logic
	);
	end component pcie_core;

   component altgx_reconfig is
      port (
          reconfig_clk     : in std_logic ;
          reconfig_fromgxb : in std_logic_vector (4 downto 0);
          busy             : out std_logic ;
          reconfig_togxb   : out std_logic_vector (3 downto 0)
	   );
   end component altgx_reconfig;

   type reg_data_out_type is array (0 to can_fd_instances_number - 1) of
	                                 std_logic_vector(31 downto 0);

   signal reg_data_in      : std_logic_vector(31 downto 0);
   signal reg_data_out     : reg_data_out_type;
   signal reg_addr         : std_logic_vector(COMP_TYPE_ADRESS_HIGHER downto 0);
   signal reg_be           : std_logic_vector(3 downto 0);
   signal reg_rden         : std_logic;
   signal reg_wren         : std_logic;
   signal reg_cs           : std_logic_vector(can_fd_instances_number - 1 downto 0);
   signal irq_vec          : std_logic_vector(can_fd_instances_number - 1 downto 0);
   constant irq_vec0       : std_logic_vector(can_fd_instances_number - 1 downto 0)
	                                           := (others => '0');
   signal inst_sel         : natural;
	signal timestamp        : std_logic_vector(63 downto 0);
   signal clk_sys          : std_logic;
   signal irq              : std_logic;
   signal bus_rw           : std_logic;
   signal bus_en           : std_logic;
   signal bus_addr         : std_logic_vector(15 downto 0);
   signal bus_ack          : std_logic;
   signal bus_data_rd      : std_logic_vector(31 downto 0);
   signal bus_data_wr      : std_logic_vector(31 downto 0);

   signal can_tx_combined  : std_logic;
   signal can_tx_vec       : std_logic_vector(can_fd_instances_number - 1 downto 0);
   signal can_rx_vec       : std_logic_vector(can_fd_instances_number - 1 downto 0);

   signal clk_50           : std_logic ;
   signal reconfig_fromgxb : std_logic_vector (4 downto 0);
   signal reconfig_busy    : std_logic ;
   signal reconfig_togxb   : std_logic_vector (3 downto 0);

-- {ALTERA_COMPONENTS_BEGIN} DO NOT REMOVE THIS LINE!
-- {ALTERA_COMPONENTS_END} DO NOT REMOVE THIS LINE!
begin
-- {ALTERA_INSTANTIATION_BEGIN} DO NOT REMOVE THIS LINE!
-- {ALTERA_INSTANTIATION_END} DO NOT REMOVE THIS LINE!

	pcie_core_inst : component pcie_core
		port map (
			clk_25_clk                                              => clk25b,			-- clk_25.clk
			reset_n_reset_n                                         => reset_n,       --  reset.reset_n

			-- has to be 100 or 125 MHz as defined in altera_pcie_hard_ip properties
			pcie_hard_ip_0_refclk_export                            => pcie_ref_clk,   -- pcie_hard_ip_0_refclk.export
			pcie_hard_ip_0_rx_in_rx_datain_0                        => pcie_rx0,       -- pcie_hard_ip_0_rx_in.rx_datain_0
			pcie_hard_ip_0_tx_out_tx_dataout_0                      => pcie_tx0,       -- pcie_hard_ip_0_tx_out.tx_dataout_0
			pcie_hard_ip_0_pcie_rstn_export                         => pcie_reset_n,   -- pcie_hard_ip_0_pcie_rstn.export

			pcie_hard_ip_0_reconfig_busy_busy_altgxb_reconfig       => reconfig_busy,  --  pcie_hard_ip_0_reconfig_busy.busy_altgxb_reconfig
			pcie_hard_ip_0_reconfig_fromgxb_0_data                  => reconfig_fromgxb, -- data
			pcie_hard_ip_0_reconfig_togxb_data                      => reconfig_togxb, -- data

			external_bus_interface_acknowledge => bus_ack,       -- external_bus_interface.acknowledge
			external_bus_interface_irq         => irq,           -- .irq
			external_bus_interface_address     => bus_addr,      -- .address
			external_bus_interface_bus_enable  => bus_en,        -- .bus_enable
			external_bus_interface_byte_enable => reg_be,        -- .byte_enable
			external_bus_interface_rw          => bus_rw ,       -- .rw
			external_bus_interface_write_data  => bus_data_wr,   -- .write_data
			external_bus_interface_read_data   => bus_data_rd,  -- .read_data

			clk_50_clk                         => clk_50,
         clk_100_clk                        => clk_sys,
			pcie_core_clk_clk                  => open
		);

   altgx_reconfig_inst : component altgx_reconfig
      port map (
          reconfig_clk     => clk_50,
          reconfig_fromgxb => reconfig_fromgxb,
          busy             => reconfig_busy,
          reconfig_togxb   => reconfig_togxb
	   );

   can_fd_instances_for : for k in 0 to can_fd_instances_number-1 generate
   begin
       reg_cs(k) <= '1' when k = inst_sel
		              else '0';

       can_fd_inst : CAN_top_level
            generic map (
                use_logger      => k = 0,
                rx_buffer_size  => 2048,
                use_sync        => true,
                sup_filtA       => true,
                sup_filtB       => true,
                sup_filtC       => true,
                sup_range       => true,
                logger_size     => 32
            )
            port map (
                clk_sys         => clk_sys,
                res_n           => reset_n,

                data_in         => reg_data_in,
                data_out        => reg_data_out(k),
                adress          => reg_addr,
                scs             => reg_cs(k),
                srd             => reg_rden,
                swr             => reg_wren,
                sbe             => reg_be,

                int             => irq_vec(k),

                CAN_tx          => can_tx_vec(k),
                CAN_rx          => can_rx_vec(k),

                time_quanta_clk => open,
                timestamp       => timestamp
            );
    end generate can_fd_instances_for;

	 ack_delay : process (reset_n, clk_sys)
    begin
        if (reset_n = '0') then
             bus_ack <= '0';
        elsif (rising_edge(clk_sys)) then
		       bus_ack <= bus_en and not bus_ack;
        end if;
    end process;

	 timestamp_counter : process (reset_n, clk_sys)
    begin
        if (reset_n = '0') then
             timestamp <= (OTHERS => '0');
        elsif (rising_edge(clk_sys)) then
		       timestamp <= std_logic_vector(to_unsigned(
				                   (to_integer(unsigned(timestamp)) + 1),
								    timestamp'length));
        end if;
    end process;

    reg_rden <= bus_en and bus_rw and not bus_ack;
    reg_wren <= bus_en and not bus_rw and not bus_ack;

    reg_addr(COMP_TYPE_ADRESS_HIGHER downto COMP_TYPE_ADRESS_LOWER) <=
        CAN_COMPONENT_TYPE;

    reg_addr(ID_ADRESS_HIGHER downto ID_ADRESS_LOWER) <=
        std_logic_vector(to_unsigned(1, 4));

    -- forward only aligned addresses
    -- (the bridge/CPU will then pick the bytes itself)
    reg_addr(ID_ADRESS_LOWER - 1 downto 2) <=
        bus_addr(ID_ADRESS_LOWER - 1 downto 2);

    reg_addr(1 downto 0) <= (others => '0');

	 irq <= '1' when irq_vec /= irq_vec0 else '0';

    inst_sel <= to_integer(unsigned(bus_addr(15 downto 14)));

	 reg_data_in <= bus_data_wr;

	 bus_data_rd <= reg_data_out(inst_sel);

	 can_tx_combine_process : process (can_tx_vec)
    variable can_tx_and  : std_logic;
    begin
        can_tx_and := '1';
        can_tx_and_for : for k in 0 to can_tx_vec'length-1 loop
            can_tx_and := can_tx_and and can_tx_vec(k);
        end loop can_tx_and_for;
        can_tx_combined <= can_tx_and;
    end process can_tx_combine_process;

    can_tx <= can_tx_combined when can_fd_local_feedback = '0' else '1';

    can_rx_vec <= (others => can_rx) when can_fd_local_feedback = '0' else
	                                      (others => can_tx_combined);

end;
