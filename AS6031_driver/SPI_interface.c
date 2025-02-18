#include "SPI_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define SPI_CHANNEL 0  // SPI channel (0 corresponds to /dev/spidev0.0)
#define SPI_SPEED 500000 // SPI speed in Hz
#define CS_GPIO 10 // GPIO pin for chip select (WiringPi numbering)

// Function to initialize SPI
void spi_init()
{
    if (wiringPiSetup() == -1)
    {
        perror("Failed to initialize WiringPi");
        exit(1);
    }

    pinMode(CS_GPIO, OUTPUT); // Set CS pin as output
    digitalWrite(CS_GPIO, HIGH); // Set CS HIGH (inactive)

    if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) < 0)
    {
        perror("Failed to initialize SPI");
        exit(1);
    }

    printf("SPI initialized!\n");
}

// Function to close SPI (Not needed in WiringPi but kept for consistency)
void spi_close()
{
    printf("SPI closed!\n");
}

// Function to set GPIO HIGH/LOW
void set_gpio(int pin, int value)
{
    digitalWrite(pin, value);
}

// Function to read GPIO value
int read_gpio(int pin)
{
    return digitalRead(pin);
}

// Function to send SPI data
void spi_write(uint8_t *data, int length)
{
    digitalWrite(CS_GPIO, LOW); // Pull CS LOW
    wiringPiSPIDataRW(SPI_CHANNEL, data, length);
    digitalWrite(CS_GPIO, HIGH); // Pull CS HIGH
}

// Function to read data over SPI
void spi_read(uint8_t *data, int length)
{
    digitalWrite(CS_GPIO, LOW); // Pull CS LOW
    wiringPiSPIDataRW(SPI_CHANNEL, data, length);
    digitalWrite(CS_GPIO, HIGH); // Pull CS HIGH
}

void Write_Opcode(uint8_t one_byte)
{
  /* 1. Put SSN low - Activate */
  set_gpio(CS_GPIO, 0);

  printf("CS_GPIO = %d !\n", read_gpio(CS_GPIO));
  fflush(stdout);
  /* 2. Transmit register address */
  spi_write(&one_byte, 1);

  set_gpio(CS_GPIO, 1);
  printf("CS_GPIO = %d !\n", read_gpio(CS_GPIO));
  fflush(stdout);
  return;
}

/**
 * @brief  Write one double word.
 * @param  opcode (byte)
 * @param  address (byte)
 * @param  dword (double word)
 * @retval none
 */
void Write_Dword(uint8_t opcode, uint8_t address, uint32_t dword)
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

  spi_write(spiTX, 6);

  return;
}

/**
 * @brief  Write one byte.
 * @param  opcode (byte)
 * @param  address (two bytes)
 * @param  byte (byte)
 * @retval none
 */
void Write_Byte2(uint8_t opcode, uint16_t address, uint8_t byte)
{
  uint8_t spiTX[4];

  spiTX[0] = opcode;
  spiTX[1] = address >> 8; // highest byte
  spiTX[2] = address;      // lowest byte
  spiTX[3] = byte;

  spi_write(spiTX, 4);

  return;
}

/**
 * @brief  Read double word.
 * @param  opcode (byte)
 * @param  address (byte)
 * @retval 32-bit value
 */
uint32_t Read_Dword(uint8_t rd_opcode, uint8_t address)
{
  uint8_t spiTX[2];
  uint8_t spiRX[4];
  uint32_t temp_u32 = 0;

  spiTX[0] = rd_opcode;
  spiTX[1] = address;

  spi_write(spiTX, 2);

  spi_read(spiRX, 4);

  for (int i = 0; i < 4; i++)
  {
    printf("spiRX ==%d, i = %d!\n", temp_u32, i);
    fflush(stdout);
  }

  /*Concatenate of bytes (from MSB to LSB) */
  temp_u32 = (spiRX[0] << 24) + (spiRX[1] << 16) + (spiRX[2] << 8) + (spiRX[3]);
  printf("temp_u32 ==%d!\n", temp_u32);
  fflush(stdout);
  return temp_u32;
}

uint32_t Read_Dword_Bits(uint8_t rd_opcode, uint8_t address, uint8_t msbit, uint8_t lsbit)
{
  // #define _DEBUGGGING_FUNCTION
  printf("Read_Dword_Bits!\n");
  fflush(stdout);
  uint32_t address_content = 0;
  uint32_t bit_amount = 0;
  uint32_t bit_mask = 0;
  uint32_t temp_u32 = 0;

  /* out of range [31:0] */
  if (msbit > 31)
    msbit = 31;
  if (lsbit > 31)
    lsbit = 31;

  if (lsbit > msbit)
    lsbit = msbit;

  /* build the mask */
  bit_amount = msbit - lsbit;
  for (int i = 0; i < bit_amount + 1; i++)
  {
    bit_mask <<= 1;
    bit_mask += 1;
  }
  bit_mask <<= lsbit;

  /* read the register content */
  address_content = Read_Dword(rd_opcode, address);
  printf("address_content ==%d!\n", address_content);
  fflush(stdout);
  temp_u32 = (address_content & bit_mask) >> lsbit;

#ifdef _DEBUGGGING_FUNCTION
  /* for debugging */
  puts("Read_Dword_Bits");
  printf(" RD opcode = 0x%02X\taddress = 0x%02X\n", rd_opcode, address);
  printf(" msb = %u\tlsb = %u\n", msbit, lsbit);
  printf(" read content (before) = 0x%08X\tread content (after) = 0x%08X\n", address_content, temp_u32);
  printf(" RD bit_mask = 0x%08X\n", bit_mask);
#endif

#undef _DEBUGGGING_FUNCTION
  printf("temp_u32 ==%d!\n", temp_u32);
  fflush(stdout);
  return temp_u32;
}

/**
 * @brief  Write two bytes.
 * @param  byte1 (e.g. opcode RC_MT_REQ)
 * @param  byte2 (e.g. request EC_MT_REQ_BITx)
 * @retval none
 */
void Write_Opcode2(uint8_t byte1, uint8_t byte2)
{
  uint8_t spiTX[2];

  spiTX[0] = byte1;
  spiTX[1] = byte2;

  spi_write(spiTX, 2);


  return;
}
