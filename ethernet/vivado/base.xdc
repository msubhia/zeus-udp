# 156.25 MHz USER_MGT_SI570_CLOCK - CMAC input clock
set_property PACKAGE_PIN AA34      [get_ports "diff_clock_rtl_clk_n"] ;# Bank 128 - MGTREFCLK0N_128 RFSoC4x2 GT CLK
set_property PACKAGE_PIN AA33      [get_ports "diff_clock_rtl_clk_p"] ;# Bank 128 - MGTREFCLK0P_128 RFSoC4x2 GT CLK


## USER LEDS
set_property PACKAGE_PIN AR11 [ get_ports "led[0]" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "led[0]" ]

set_property PACKAGE_PIN AW10 [ get_ports "led[1]" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "led[1]" ]

set_property PACKAGE_PIN AT11 [ get_ports "led[2]" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "led[2]" ]

set_property PACKAGE_PIN AU10 [ get_ports "led[3]" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "led[3]" ]

## USER SLIDE SWITCH
set_property PACKAGE_PIN AN13 [ get_ports "sw[0]" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "sw[0]" ]

set_property PACKAGE_PIN AU12 [ get_ports "sw[1]" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "sw[1]" ]

set_property PACKAGE_PIN AW11 [ get_ports "sw[2]" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "sw[2]" ]

set_property PACKAGE_PIN AV11 [ get_ports "sw[3]" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "sw[3]" ]


set_property BITSTREAM.CONFIG.UNUSEDPIN PULLUP [current_design]
set_property BITSTREAM.CONFIG.OVERTEMPSHUTDOWN ENABLE [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]



## QSFP PHY LAYER CONTROL
set_property PACKAGE_PIN AM22 [ get_ports "qsfp_intl_ls" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "qsfp_intl_ls" ]

set_property PACKAGE_PIN AL21 [ get_ports "qsfp_resetl_ls" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "qsfp_resetl_ls" ]

set_property PACKAGE_PIN AN22 [ get_ports "qsfp_lpmode_ls" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "qsfp_lpmode_ls" ]

set_property PACKAGE_PIN AK22 [ get_ports "qsfp_modsell_ls" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "qsfp_modsell_ls" ]

