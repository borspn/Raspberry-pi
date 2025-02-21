#include <stdint.h>
#include <pigpio.h>

#define SPI_CHANNEL 0     // Use SPI channel 0
#define SPI_SPEED 500000 // SPI speed (500 kHz)
#define CS_GPIO 25        // Chip Select GPIO pin

#define LOW 0
#define HIGH 1
#define PUT_SSN_LOW gpioWrite(CS_GPIO, LOW)
#define PUT_SSN_HIGH gpioWrite(CS_GPIO, HIGH)

// Initializes the SPI interface
void spi_init();
// Closes the SPI interface
void spi_close();
// Writes a single byte as an opcode
void Write_Opcode(uint8_t one_byte);
// Writes a 32-bit word to the specified address using the given opcode
void Write_Dword(uint8_t opcode, uint8_t address, uint32_t dword);
// Reads a 32-bit word from a specified address using the given read opcode
uint32_t Read_Dword(uint8_t rd_opcode, uint8_t address);
// Reads a specified number of bits from a 32-bit word at a specified address using the given read opcode
uint32_t Read_Dword_Bits(uint8_t rd_opcode, uint8_t address, uint8_t msbit, uint8_t lsbit);
// Writes two bytes as an opcode
void Write_Opcode2(uint8_t byte1, uint8_t byte2);