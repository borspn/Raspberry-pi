#include "SPI_interface.h"
#include <stdio.h>
#include <stdlib.h>

int spi_handle;

/**
 * @brief Initialize SPI communication using pigpio.
 */
void spi_init()
{
    if (gpioInitialise() < 0)
    {
        fprintf(stderr, "Failed to initialize pigpio!\n");
        exit(1);
    }

    // Open SPI device
    spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
    if (spi_handle < 0)
    {
        fprintf(stderr, "Failed to open SPI device!\n");
        gpioTerminate();
        exit(1);
    }

    // Set CS GPIO as output and default HIGH (inactive)
    gpioSetMode(CS_GPIO, PI_OUTPUT);
    PUT_SSN_HIGH;

    printf("SPI initialized using pigpio!\n");
}

/**
 * @brief Close SPI and release GPIO resources.
 */
void spi_close()
{
    spiClose(spi_handle);
    gpioTerminate();
    printf("SPI closed and GPIO released.\n");
}

static void spiReceive(int spi_handle, char *dataTx, char *dataRx, int len)
{
    PUT_SSN_LOW;
    spiXfer(spi_handle, dataTx, dataRx, len);
    PUT_SSN_HIGH;
}

static void spiSend(uint8_t *data, int len)
{
    PUT_SSN_LOW;
    spiWrite(spi_handle, (char *)data, len);
    PUT_SSN_HIGH;
}

/**
 * @brief Write a single opcode byte via SPI.
 */
void Write_Opcode(uint8_t one_byte)
{
    spiSend(&one_byte, 1); // Send opcode
}

/**
 * @brief Write one double word (4 bytes).
 */
void Write_Dword(uint8_t opcode, uint8_t address, uint32_t dword)
{
    uint8_t spiTX[6];

    spiTX[0] = opcode;
    spiTX[1] = address;
    spiTX[2] = (dword >> 24) & 0xFF;
    spiTX[3] = (dword >> 16) & 0xFF;
    spiTX[4] = (dword >> 8) & 0xFF;
    spiTX[5] = dword & 0xFF;

    spiSend(spiTX, 6);
}

/**
 * @brief Write one byte with a 16-bit address.
 */
void Write_Byte2(uint8_t opcode, uint16_t address, uint8_t byte)
{
    uint8_t spiTX[4];

    spiTX[0] = opcode;
    spiTX[1] = (address >> 8) & 0xFF;
    spiTX[2] = address & 0xFF;
    spiTX[3] = byte;

    spiSend(spiTX, 4);
}

/**
 * @brief Read a double word (4 bytes) via SPI.
 */
uint32_t Read_Dword(uint8_t rd_opcode, uint8_t address)
{
    uint8_t spiTX[2] = {rd_opcode, address};
    uint8_t spiRX[6] = {0};

    spiReceive(spi_handle, (char *)spiTX, (char *)spiRX, 6); // Send opcode/address, receive data

    uint32_t temp_u32 = (spiRX[2] << 24) | (spiRX[3] << 16) | (spiRX[4] << 8) | spiRX[5];

    return temp_u32;
}

/**
 * @brief Read specific bits from a double word.
 */
uint32_t Read_Dword_Bits(uint8_t rd_opcode, uint8_t address, uint8_t msbit, uint8_t lsbit)
{

    if (msbit > 31)
        msbit = 31;
    if (msbit >= 32)
        msbit = 31;
    if (lsbit >= 32)
        lsbit = 31;

    uint32_t address_content = Read_Dword(rd_opcode, address);
    uint32_t bit_mask = (1U << (msbit - lsbit + 1)) - 1;
    uint32_t temp_u32 = (address_content >> lsbit) & bit_mask;

    return temp_u32;
}

/**
 * @brief Write two bytes via SPI.
 */
void Write_Opcode2(uint8_t byte1, uint8_t byte2)
{
    uint8_t spiTX[2] = {byte1, byte2};
    spiSend(spiTX, 2);
}
