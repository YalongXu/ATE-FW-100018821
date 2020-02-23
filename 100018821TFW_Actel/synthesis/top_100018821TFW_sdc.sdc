# Top Level Design Parameters

# Clocks

create_clock -period 25.000000 -waveform {0.000000 12.500000} SYSCLK
create_clock -period 31.250000 -waveform {0.000000 15.625000} PCI_CLK2
create_clock -period 50.000000 -waveform {0.000000 25.000000} osc_count/pci_clk_div/CLK_OUT:Q
create_clock -period 1000.000000 -waveform {0.000000 500.000000} LedCont/Clk10Hz/CLK_OUT:Q
create_clock -period 50.000000 -waveform {0.000000 25.000000} ad1_mod/ram_wr_clk:Q
create_clock -period 50.000000 -waveform {0.000000 25.000000} ad1_mod/sclk/CLK_OUT:Q
create_clock -period 50.000000 -waveform {0.000000 25.000000} ad2_mod/ram_wr_clk:Q
create_clock -period 50.000000 -waveform {0.000000 25.000000} ad2_mod/sclk/CLK_OUT:Q
create_clock -period 50.000000 -waveform {0.000000 25.000000} LiftDac/tx_clk_mod/CLK_OUT:Q
create_clock -period 50.000000 -waveform {0.000000 25.000000} rs485_mod/pci_clk_div/CLK_OUT:Q
create_clock -period 50.000000 -waveform {0.000000 25.000000} ilim_dac_mod/tx_clk_mod/CLK_OUT:Q
create_clock -period 50.000000 -waveform {0.000000 25.000000} {MEL_XTRA[1]}

# False Paths Between Clocks


# False Path Constraints


# Maximum Delay Constraints


# Multicycle Constraints


# Virtual Clocks
# Output Load Constraints
# Driving Cell Constraints
# Wire Loads
# set_wire_load_mode top

# Other Constraints
