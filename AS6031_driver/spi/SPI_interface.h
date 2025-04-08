
#include <stdint.h>
#include <gpiod.h>
#include <fcntl.h>

#define LOW 0
#define HIGH 1
#define PUT_SSN_LOW gpiod_line_set_value(cs_line, LOW);
#define PUT_SSN_HIGH gpiod_line_set_value(cs_line, HIGH);

extern int spi_handle;
extern struct gpiod_chip *chip;
extern struct gpiod_line *cs_line;

// Initializes the SPI interface
void spi_init(uint8_t csGpio, uint32_t spiSpeed);
// Writes a single byte as an opcode
void Write_Opcode(char one_byte);
// Writes a 32-bit word to the specified address using the given opcode
void Write_Dword(char opcode, uint8_t address, uint32_t dword);
// Reads a 32-bit word from a specified address using the given read opcode
uint32_t Read_Dword(char rd_opcode, uint8_t address);
// Writes two bytes as an opcode
void Write_Opcode2(char byte1, char byte2);

void Write_Register_Auto_Incr(uint8_t opcode, uint8_t from_addr, uint32_t *dword_array, int to_addr);