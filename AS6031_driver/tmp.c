#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>

#define SPI_DEVICE "/dev/spidev0.0" // Use spidev0.1 for CE1
#define CS_GPIO 17 // Use any free GPIO pin for manual Chip Select

int spi_fd; // SPI file descriptor

// Function to write to sysfs GPIO
void set_gpio(int pin, int value) {
    char path[50];
    sprintf(path, "/sys/class/gpio/gpio%d/value", pin);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to set GPIO");
        return;
    }
    write(fd, value ? "1" : "0", 1);
    close(fd);
}

// Function to export and set up GPIO for CS
void init_gpio(int pin) {
    char path[50];
    sprintf(path, "/sys/class/gpio/export");
    int fd = open(path, O_WRONLY);
    if (fd >= 0) {
        char buf[3];
        sprintf(buf, "%d", pin);
        write(fd, buf, strlen(buf));
        close(fd);
    }
    
    sprintf(path, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (fd >= 0) {
        write(fd, "out", 3);
        close(fd);
    }
}

// Function to initialize SPI
void spi_init() {
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        perror("Failed to open SPI device");
        exit(1);
    }

    uint8_t mode = SPI_MODE_0 | SPI_NO_CS; // Use manual CS
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);

    uint32_t speed = 500000; // 500 kHz
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    init_gpio(CS_GPIO); // Set up CS pin
    set_gpio(CS_GPIO, 1); // Default HIGH (inactive)
}

// Function to send SPI data
void spi_send(uint8_t *data, int length) {
    set_gpio(CS_GPIO, 0); // Pull CS LOW
    struct spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long)data,
        .rx_buf = 0,
        .len = length,
        .speed_hz = 500000,
        .bits_per_word = 8,
        .delay_usecs = 0
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer);
    set_gpio(CS_GPIO, 1); // Pull CS HIGH
}

// Main function
int main() {
    spi_init();

    uint8_t data[] = {0xAA, 0xBB}; // Data to send
    spi_send(data, sizeof(data));

    close(spi_fd);
    return 0;
}
