#include <stdint.h>
#include <pigpio.h>

#define SPI_CHANNEL 0     // Use SPI channel 0
#define SPI_SPEED 1000000 // SPI speed (1 MHz)
#define CS_GPIO 25        // Chip Select GPIO pin

#define LOW 0
#define HIGH 1
#define PUT_SSN_LOW gpioWrite(CS_GPIO, LOW)
#define PUT_SSN_HIGH gpioWrite(CS_GPIO, HIGH)

extern void Write_Opcode(uint8_t one_byte);
void set_gpio(int pin, int value);
void test();
void spi_init();
void spi_close();
void Write_Byte2(uint8_t opcode, uint16_t address, uint8_t byte);
void Write_Dword(uint8_t opcode, uint8_t address, uint32_t dword);
uint32_t Read_Dword(uint8_t rd_opcode, uint8_t address);
uint32_t Read_Dword_Bits(uint8_t rd_opcode, uint8_t address, uint8_t msbit, uint8_t lsbit);
void Write_Opcode2(uint8_t byte1, uint8_t byte2);
