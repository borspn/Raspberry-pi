#include "SPI_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


#define SPI_SPEED_HZ 500000  // Set SPI clock speed to 500kHz

int spi_handle;
struct gpiod_chip *chip;
struct gpiod_line *cs_line;
struct gpiod_line *irq_line;
/**
 * @brief Initialize SPI communication using libgpiod.
 */
void spi_init()
{
    // Open GPIO chip
    chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip)
    {
        fprintf(stderr, "Failed to open GPIO chip!\n");
        exit(1);
    }

    // Open CS GPIO line
    cs_line = gpiod_chip_get_line(chip, CS_GPIO);
    if (!cs_line)
    {
        fprintf(stderr, "Failed to get CS GPIO line!\n");
        gpiod_chip_close(chip);
        exit(1);
    }

    // Set CS GPIO as output and default HIGH (inactive)
    if (gpiod_line_request_output(cs_line, "spi", 1) < 0)
    {
        fprintf(stderr, "Failed to set CS GPIO as output!\n");
        gpiod_chip_close(chip);
        exit(1);
    }

    // Open SPI device
    spi_handle = open("/dev/spidev0.0", O_RDWR);
    if (spi_handle < 0)
    {
        fprintf(stderr, "Failed to open SPI device!\n");
        gpiod_chip_close(chip);
        exit(1);
    }

    uint32_t speed = SPI_SPEED_HZ;
    if (ioctl(spi_handle, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
    {
        fprintf(stderr, "Failed to set SPI speed!\n");
        close(spi_handle);
        gpiod_chip_close(chip);
        exit(1);
    }

    printf("SPI initialized using libgpiod!\n");
}

/**
 * @brief Close SPI and release GPIO resources.
 */
void spi_close()
{
    close(spi_handle);
    gpiod_chip_close(chip);
    printf("SPI closed and GPIO released.\n");
}

/**
 * @brief Write a single opcode byte via SPI.
 */
void Write_Opcode(char one_byte)
{
    gpiod_line_set_value(cs_line, 0);
    write(spi_handle, &one_byte, 1); // Send opcode
    gpiod_line_set_value(cs_line, 1);
}

/**
 * @brief Write one double word (4 bytes).
 */
void Write_Dword(char opcode, uint8_t address, uint32_t dword)
{
    uint8_t spiTX[6];
    uint32_t temp_u32 = 0;

    spiTX[0] = opcode;
    spiTX[1] = address;
    temp_u32 = dword;
    spiTX[2] = temp_u32 >> 24;
    spiTX[3] = temp_u32 >> 16;
    spiTX[4] = temp_u32 >> 8;
    spiTX[5] = temp_u32;

    gpiod_line_set_value(cs_line, 0);
    /* 2. Transmit register address */
    write(spi_handle, spiTX, 6);
    gpiod_line_set_value(cs_line, 1);
}

/**
 * @brief Write one byte with a 16-bit address.
 */
void Write_Byte2(char opcode, uint16_t address, uint8_t byte)
{
    char spiTX[4];

    spiTX[0] = opcode;
    spiTX[1] = (address >> 8) & 0xFF;
    spiTX[2] = address & 0xFF;
    spiTX[3] = byte;

    gpiod_line_set_value(cs_line, 0);
    write(spi_handle, spiTX, 4);
    gpiod_line_set_value(cs_line, 1);
}

uint32_t Read_Dword(char rd_opcode, uint8_t address)
{
    uint8_t spiTX[2];
    uint8_t spiRX[4];
    uint32_t temp_u32 = 0;

    spiTX[0] = rd_opcode;
    spiTX[1] = address;
    gpiod_line_set_value(cs_line, 0);
    write(spi_handle, spiTX, 2); // Send opcode/address, receive data
    read(spi_handle, spiRX, 4);
    gpiod_line_set_value(cs_line, 1);
    temp_u32 = (spiRX[0] << 24) + (spiRX[1] << 16) + (spiRX[2] << 8) + (spiRX[3]);

    return temp_u32;
}

/**
 * @brief Read specific bits from a double word.
 */
uint32_t Read_Dword_Bits(char rd_opcode, uint8_t address, uint8_t msbit, uint8_t lsbit)
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
void Write_Opcode2(char byte1, char byte2)
{
    char spiTX[2] = {byte1, byte2};
    gpiod_line_set_value(cs_line, 0);
    write(spi_handle, spiTX, 2);
    gpiod_line_set_value(cs_line, 1);
}

void Write_Register_Auto_Incr(uint8_t opcode, uint8_t from_addr, uint32_t *dword_array, int to_addr)
{
    uint8_t spiTX[4];
    uint32_t temp_u32 = 0;

    spiTX[0] = opcode;
    spiTX[1] = from_addr;

    gpiod_line_set_value(cs_line, 0);
    /* 2.a Transmit register address */
    write(spi_handle, spiTX, 2);

    /* 2.b Transmit register address incrementally */
    for (int i = from_addr; i <= to_addr; i++)
    {
        temp_u32 = *dword_array;
        spiTX[0] = temp_u32 >> 24;
        spiTX[1] = temp_u32 >> 16;
        spiTX[2] = temp_u32 >> 8;
        spiTX[3] = temp_u32;

        write(spi_handle, spiTX, 4);

        dword_array++;
    }
    gpiod_line_set_value(cs_line, 1);
}