
#include <stdint.h>
#include <gpiod.h>
#include <fcntl.h>
#include <unistd.h>

#define SPI_CHANNEL 0     // Use SPI channel 0
#define SPI_SPEED 1000000 // SPI speed (1 MHz)
#define CS_GPIO 25        // Chip Select GPIO pin

#define LOW 0
#define HIGH 1
#define PUT_SSN_LOW gpioWrite(CS_GPIO, LOW)
#define PUT_SSN_HIGH gpioWrite(CS_GPIO, HIGH)

extern int spi_handle;
extern struct gpiod_chip *chip;
extern struct gpiod_line *cs_line;
extern struct gpiod_line *irq_line;

// Initializes the SPI interface
void spi_init();
// Closes the SPI interface
void spi_close();
// Writes a single byte as an opcode
void Write_Opcode(char one_byte);
// Writes a 32-bit word to the specified address using the given opcode
void Write_Dword(char opcode, uint8_t address, uint32_t dword);
// Writes a byte to a specified address using the given opcode
void Write_Byte2(char opcode, uint16_t address, uint8_t byte);
// Reads a 32-bit word from a specified address using the given read opcode
uint32_t Read_Dword(char rd_opcode, uint8_t address);
// Reads a specified number of bits from a 32-bit word at a specified address using the given read opcode
uint32_t Read_Dword_Bits(char rd_opcode, uint8_t address, uint8_t msbit, uint8_t lsbit);
// Writes two bytes as an opcode
void Write_Opcode2(char byte1, char byte2);

void Write_Register_Auto_Incr(uint8_t opcode, uint8_t from_addr, uint32_t *dword_array, int to_addr);