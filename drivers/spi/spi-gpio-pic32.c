/*
 * Hard coded (faster) bitbang GPIO driver for PIC32MZ board.
 */
#define DRIVER_NAME	"spi_gpio_pic32mz"
#define	SPI_MISO_GPIO	234
#define	SPI_MOSI_GPIO	163
#define	SPI_SCK_GPIO	36
#define	SPI_N_CHIPSEL	1
#include "spi-gpio.c"
