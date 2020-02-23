# Synopsys, Inc. constraint file
# H:/HE1.6/Hardware/FPGA/100018821-TFW/build/constraints/top_100018821TFW_synth.sdc
# Written on Tue Apr 05 07:35:45 2011
# by Synplify Pro, D-2009.12A Scope Editor

#
# Collections
#

#
# Clocks
#
define_clock   {SYSCLK}                          -name {SYSCLK}          -freq 40 -clockgroup default_clkgroup_0
define_clock   {PCI_CLK2}                        -name {PCI_CLK2}        -freq 32 -clockgroup default_clkgroup_1
define_clock   {osc_count.pci_clk_div.CLK_OUT}   -name {divided_pci_clk} -freq 20 -clockgroup default_clkgroup_2
define_clock   {LedCont.Clk10Hz.CLK_OUT}         -name {TenHz}           -freq 1  -clockgroup default_clkgroup_3
define_clock   {ad1_mod.ram_wr_clk}              -name {ad1_ram_wr_clk}  -freq 20 -clockgroup default_clkgroup_4
define_clock   {ad1_mod.sclk.CLK_OUT}            -name {ad1_clk_sd}      -freq 20 -clockgroup default_clkgroup_5
define_clock   {ad2_mod.ram_wr_clk}              -name {ad2_ram_wr_clk}  -freq 20 -clockgroup default_clkgroup_6
define_clock   {ad2_mod.sclk.CLK_OUT}            -name {ad2_clk_sd}      -freq 20 -clockgroup default_clkgroup_7
define_clock   {LiftDac.tx_clk_mod.CLK_OUT}      -name {lift_tx_clk}     -freq 20 -clockgroup default_clkgroup_8
define_clock   {rs485_mod.pci_clk_div.CLK_OUT}   -name {data_clk}        -freq 20 -clockgroup default_clkgroup_9
define_clock   {ilim_dac_mod.tx_clk_mod.CLK_OUT} -name {ilim_tx_clk}     -freq 20 -clockgroup default_clkgroup_10
define_clock   {MEL_XTRA[1]}                     -name {mel_xtra_1}      -freq 20 -clockgroup default_clkgroup_11

#
# Clock to Clock
#

#
# Inputs/Outputs
#

#
# Registers
#

#
# Delay Paths
#

#
# Attributes
#
define_global_attribute {syn_global_buffers} {009}
define_attribute {n:SYSCLK}                          {syn_insert_buffer}  {CLKINT}
define_attribute {n:PCI_CLK2}                        {syn_insert_buffer}  {CLKINT}
define_attribute {n:PCI_RST}                         {syn_insert_buffer}  {CLKINT}
define_attribute {n:ad1_mod.sclk.CLK_OUT}            {syn_insert_buffer}  {CLKINT}
define_attribute {n:ad2_mod.sclk.CLK_OUT}            {syn_insert_buffer}  {CLKINT}
define_attribute {n:LiftDac.tx_clk_mod.CLK_OUT}      {syn_insert_buffer}  {CLKINT}
define_attribute {n:LedCont.Clk10Hz.CLK_OUT}         {syn_insert_buffer}  {CLKINT}
define_attribute {n:ilim_dac_mod.tx_clk_mod.CLK_OUT} {syn_insert_buffer}  {CLKINT}
define_attribute {n:rs485_mod.pci_clk_div.CLK_OUT}   {syn_insert_buffer}  {CLKINT}

#
# I/O Standards
#

#
# Compile Points
#

#
# Other
#
