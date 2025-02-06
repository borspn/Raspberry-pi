#include "SPI_interface.h"
/**
 * @brief  Write one byte Opcode.
 * @param  one_byte
 * @retval none
 */

void spi_write(uint8_t *data, int length)
{
    digitalWrite(GPIO_PIN_SSN, LOW); // Select the sensor
    wiringPiSPIDataRW(SPI_CHANNEL, data, length);
    digitalWrite(GPIO_PIN_SSN, HIGH); // Deselect the sensor
}

void spi_read(uint8_t *data, int length)
{
    digitalWrite(GPIO_PIN_SSN, LOW); // Select the sensor
    wiringPiSPIDataRW(SPI_CHANNEL, data, length);
    digitalWrite(GPIO_PIN_SSN, HIGH); // Deselect the sensor
}

void Write_Opcode(uint8_t one_byte)
{
    /* 1. Put SSN low - Activate */
    PUT_SSN_LOW;

    /* 2. Transmit register address */
    spi_write(&one_byte, 1);

    /* 3. Put SSN high - Deactivate */
    PUT_SSN_HIGH;

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

    /* 1. Put SSN low - Activate */
    PUT_SSN_LOW;
    /* 2. Transmit register address */
    spi_write(spiTX, 6);

    /* 3. Put SSN high - Deactivate */
    PUT_SSN_HIGH;

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

    /* 1. Put SSN low - Activate */
    PUT_SSN_LOW;

    /* 2. Transmit register address */
    spi_write(spiTX, 4);

    /* 3. Put SSN high - Deactivate */
    PUT_SSN_HIGH;

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

    /* 1. Put SSN low - Activate */
    PUT_SSN_LOW;

    /* 2. Transmit register address */
    spi_write(spiTX, 2);

    /*3. Read four bytes */
    spi_read(spiRX, 4);

    /* 4. Put SSN high - Deactivate */
    PUT_SSN_HIGH;

    /*Concatenate of bytes (from MSB to LSB) */
    temp_u32 = (spiRX[0] << 24) + (spiRX[1] << 16) + (spiRX[2] << 8) + (spiRX[3]);

    return temp_u32;
}

uint32_t Read_Dword_Bits(uint8_t rd_opcode, uint8_t address, uint8_t msbit, uint8_t lsbit)
{
//#define _DEBUGGGING_FUNCTION
  
  uint32_t address_content = 0;
  uint32_t bit_amount = 0;
  uint32_t bit_mask = 0;
  uint32_t temp_u32 = 0;
  
  /* out of range [31:0] */
  if (msbit > 31) msbit = 31;
  if (lsbit > 31) lsbit = 31;
  
  if (lsbit > msbit) lsbit = msbit;
  
  /* build the mask */
  bit_amount = msbit - lsbit;
  for (int i = 0; i < bit_amount + 1; i++) {
    bit_mask <<= 1;
    bit_mask += 1;
  }
  bit_mask <<= lsbit;
  
  /* read the register content */
  address_content = Read_Dword(rd_opcode, address);
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
      
  /* 1. Put SSN low - Activate */
  PUT_SSN_LOW;
  
  /* 2. Transmit register address */
  spi_write(spiTX, 2);
  
  /* 3. Put SSN high - Deactivate */
  PUT_SSN_HIGH;

  return;
}
