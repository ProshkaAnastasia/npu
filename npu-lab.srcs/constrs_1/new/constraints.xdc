## ============================================================
## Nexys4 DDR: nn_processor_top
## ============================================================

## Тактовый сигнал платы: 100 МГц.
set_property -dict { PACKAGE_PIN E3 IOSTANDARD LVCMOS33 } \
    [get_ports {clk_i}]

create_clock -name sys_clk -period 10.000 -waveform {0.000 5.000} \
    [get_ports {clk_i}]


## Кнопка CPU_RESETN: сброс активен при нуле.
set_property -dict { PACKAGE_PIN C12 IOSTANDARD LVCMOS33 } \
    [get_ports {rst_n_i}]


## UART RX: компьютер -> USB-UART мост -> FPGA.
set_property -dict { PACKAGE_PIN C4 IOSTANDARD LVCMOS33 } \
    [get_ports {uart_rx_i}]

## UART TX: FPGA -> USB-UART мост -> компьютер.
set_property -dict { PACKAGE_PIN D4 IOSTANDARD LVCMOS33 } \
    [get_ports {uart_tx_o}]


## LED0: конфигурация сети загружена и валидна.
set_property -dict { PACKAGE_PIN H17 IOSTANDARD LVCMOS33 } \
    [get_ports {model_valid_o}]

## LED1: процессор занят.
set_property -dict { PACKAGE_PIN K15 IOSTANDARD LVCMOS33 } \
    [get_ports {busy_o}]