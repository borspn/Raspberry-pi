#include "SPI_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>

#define SPI_CHANNEL 0  // SPI device (spidev0.0)
#define SPI_SPEED 500000 // 500 kHz
#define CS_GPIO 6 // Chip Select GPIO pin

int spi_handle;

/**
 * @brief Initialize SPI communication using pigpio.
 */
void spi_init() {
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Failed to initialize pigpio!\n");
        exit(1);
    }

    // Open SPI device
    spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
    if (spi_handle < 0) {
        fprintf(stderr, "Failed to open SPI device!\n");
        gpioTerminate();
        exit(1);
    }

    // Set CS GPIO as output and default HIGH (inactive)
    gpioSetMode(CS_GPIO, PI_OUTPUT);
    gpioWrite(CS_GPIO, 1);

    printf("SPI initialized using pigpio!\n");
}

/**
 * @brief Close SPI and release GPIO resources.
 */
void spi_close() {
    spiClose(spi_handle);
    gpioTerminate();
    printf("SPI closed and GPIO released.\n");
}


void test(){
  gpioWrite(CS_GPIO, 0);

}
/**
 * @brief Set GPIO value (HIGH or LOW).
 */
void set_gpio(int pin, int value) {
    gpioWrite(pin, value);
}

/**
 * @brief Read GPIO value (HIGH or LOW).
 */
int read_gpio(int pin) {
  gpioWrite(CS_GPIO, 1);
  printf("1");
  fflush(stdout);
  sleep(3);
  gpioWrite(CS_GPIO, 0);
  printf("0");
  fflush(stdout);

}

/**
 * @brief Write a single opcode byte via SPI.
 */
void Write_Opcode(uint8_t one_byte) {
    gpioWrite(CS_GPIO, 0);  // Activate CS (Low)
    printf("CS_GPIO = %d !\n", gpioRead(CS_GPIO));
    fflush(stdout);

    spiWrite(spi_handle, &one_byte, 1);  // Send opcode

    gpioWrite(CS_GPIO, 1);  // Deactivate CS (High)
    printf("CS_GPIO = %d !\n", gpioRead(CS_GPIO));
    fflush(stdout);
}

/**
 * @brief Write one double word (4 bytes).
 */
void Write_Dword(uint8_t opcode, uint8_t address, uint32_t dword) {
    uint8_t spiTX[6];

    spiTX[0] = opcode;
    spiTX[1] = address;
    spiTX[2] = (dword >> 24) & 0xFF;
    spiTX[3] = (dword >> 16) & 0xFF;
    spiTX[4] = (dword >> 8) & 0xFF;
    spiTX[5] = dword & 0xFF;

    spiWrite(spi_handle, spiTX, 6);
}

/**
 * @brief Write one byte with a 16-bit address.
 */
void Write_Byte2(uint8_t opcode, uint16_t address, uint8_t byte) {
    uint8_t spiTX[4];

    spiTX[0] = opcode;
    spiTX[1] = (address >> 8) & 0xFF;
    spiTX[2] = address & 0xFF;
    spiTX[3] = byte;

    spiWrite(spi_handle, spiTX, 4);
}

/**
 * @brief Read a double word (4 bytes) via SPI.
 */
uint32_t Read_Dword(uint8_t rd_opcode, uint8_t address) {
    uint8_t spiTX[2] = { rd_opcode, address };
    uint8_t spiRX[6] = { 0 };

    spiXfer(spi_handle, spiTX, spiRX, 6);  // Send opcode/address, receive data

    uint32_t temp_u32 = (spiRX[2] << 24) | (spiRX[3] << 16) | (spiRX[4] << 8) | spiRX[5];

    printf("temp_u32 == %d!\n", temp_u32);
    fflush(stdout);
    return temp_u32;
}

/**
 * @brief Read specific bits from a double word.
 */
uint32_t Read_Dword_Bits(uint8_t rd_opcode, uint8_t address, uint8_t msbit, uint8_t lsbit) {
    printf("Read_Dword_Bits!\n");
    fflush(stdout);

    if (msbit > 31) msbit = 31;
    if (lsbit > 31) lsbit = 31;
    if (lsbit > msbit) lsbit = msbit;

    uint32_t address_content = Read_Dword(rd_opcode, address);
    uint32_t bit_mask = (1U << (msbit - lsbit + 1)) - 1;
    uint32_t temp_u32 = (address_content >> lsbit) & bit_mask;

    printf("temp_u32 == %d!\n", temp_u32);
    fflush(stdout);
    return temp_u32;
}

/**
 * @brief Write two bytes via SPI.
 */
void Write_Opcode2(uint8_t byte1, uint8_t byte2) {
    uint8_t spiTX[2] = { byte1, byte2 };
    spiWrite(spi_handle, spiTX, 2);
}
