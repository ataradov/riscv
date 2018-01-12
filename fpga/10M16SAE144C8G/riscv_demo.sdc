derive_clock_uncertainty
create_clock -name clk_i -period 50Mhz [get_ports clk_i]

set_output_delay -clock { clk_i } 5 [get_ports { led_o[7] led_o[6] led_o[5] led_o[4] led_o[3] led_o[2] led_o[1] led_o[0] }]
set_output_delay -clock { clk_i } 5 [get_ports { uart_tx_o }]

set_input_delay -clock { clk_i } 5 [get_ports { button_i[2] button_i[1] button_i[0] }]
set_input_delay -clock { clk_i } 5 [get_ports { uart_rx_i }]


