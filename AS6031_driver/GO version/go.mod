module main
require (
    github.com/warthog618/gpiod v1.1.0
    periph.io/x/conn/v3 v3.6.4
    periph.io/x/host/v3 v3.8.1
)

replace main/spi_interface => ../spi_interface
go 1.23.4
