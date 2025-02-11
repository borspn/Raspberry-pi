#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define SPI_DEVICE "/dev/spidev0.0"

int main() {
    int spi_fd;
    unsigned char tx_buffer[2] = {0xAA, 0xBB}; // Data to send
    unsigned char rx_buffer[2] = {0};

    // Open SPI device
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        perror("Failed to open SPI device");
        return -1;
    }

    // Set SPI mode
    uint8_t mode = SPI_MODE_0;
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("Failed to set SPI mode");
        close(spi_fd);
        return -1;
    }

    // Set SPI speed
    uint32_t speed = 500000; // 500 kHz
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("Failed to set SPI speed");
        close(spi_fd);
        return -1;
    }

    // SPI Transfer
    struct spi_ioc_transfer spi_transfer = {
        .tx_buf = (unsigned long)tx_buffer,
        .rx_buf = (unsigned long)rx_buffer,
        .len = 2,
        .speed_hz = speed,
        .bits_per_word = 8,
        .delay_usecs = 0,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer) < 0) {
        perror("SPI Transfer failed");
        close(spi_fd);
        return -1;
    }

    printf("Received: 0x%02X 0x%02X\n", rx_buffer[0], rx_buffer[1]);

    // Close SPI device
    close(spi_fd);
    return 0;
}
