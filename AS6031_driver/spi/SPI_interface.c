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
    spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED, 1);
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

/**
 * @brief Write a single opcode byte via SPI.
 */
void Write_Opcode(char one_byte)
{
    PUT_SSN_LOW;
    spiWrite(spi_handle, &one_byte, 1); // Send opcode
    PUT_SSN_HIGH;
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

    PUT_SSN_LOW;
    /* 2. Transmit register address */
    spiWrite(spi_handle, spiTX, 6);
    PUT_SSN_HIGH;
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

    PUT_SSN_LOW;
    spiWrite(spi_handle, spiTX, 4);
    PUT_SSN_HIGH;
}

uint32_t Read_Dword(char rd_opcode, uint8_t address)
{
    uint8_t spiTX[2];
    uint8_t spiRX[4];
    uint32_t temp_u32 = 0;

    spiTX[0] = rd_opcode;
    spiTX[1] = address;
    PUT_SSN_LOW;
    spiWrite(spi_handle, spiTX, 2); // Send opcode/address, receive data
    spiRead(spi_handle, spiRX, 4);
    PUT_SSN_HIGH;
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
} // TODO: Clarify this function with ScioSense

/**
 * @brief Write two bytes via SPI.
 */
void Write_Opcode2(char byte1, char byte2)
{
    char spiTX[2] = {byte1, byte2};
    PUT_SSN_LOW;
    spiWrite(spi_handle, spiTX, 2);
    PUT_SSN_HIGH;
}

void Write_Register_Auto_Incr(uint8_t opcode, uint8_t from_addr, uint32_t *dword_array, int to_addr)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t spiTX[4];
  uint32_t temp_u32 = 0;

  spiTX[0] = opcode;
  spiTX[1] = from_addr;

//  /* to start at expected index */
//  dword_array += from_addr; 
  
   PUT_SSN_LOW;
  /* 2.a Transmit register address */
  spiWrite(spi_handle, spiTX, 2); 
 
  /* 2.b Transmit register address incrementally */
  for (int i = from_addr; i <= to_addr; i++) {
    printf("i: %x\n", *dword_array);
    temp_u32 = *dword_array;
    spiTX[0] = temp_u32>>24;
    spiTX[1] = temp_u32>>16;
    spiTX[2] = temp_u32>>8;
    spiTX[3] = temp_u32;

    spiWrite(spi_handle, spiTX, 4); 

    dword_array++;
  }
  PUT_SSN_HIGH;
}