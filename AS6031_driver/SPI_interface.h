#include <stdint.h>

#define SPI_CHANNEL 0     // Use SPI channel 0
#define SPI_SPEED 1000000 // SPI speed (1 MHz)

// Define GPIO pins using BCM numbering
#define GPIO_PIN_MISO 9
#define GPIO_PIN_MOSI 10
#define GPIO_PIN_SCK 11
#define GPIO_PIN_SSN 8

#define SSN_PIN 8 // Define the SSN pin (BCM numbering)
#define HIGH 1
#define LOW 0
#define PUT_SSN_HIGH digitalWrite(SSN_PIN, HIGH) // Put SSN low - Activate, SSN -> CLK = < 4us
#define PUT_SSN_LOW digitalWrite(SSN_PIN, LOW)   // Put SSN low - Activate, SSN -> CLK = < 4us

extern void Write_Opcode(uint8_t one_byte);
void spi_read(uint8_t *data, int length);
void spi_write(uint8_t* data, int length);
void Write_Byte2(uint8_t opcode, uint16_t address, uint8_t byte);
void Write_Dword(uint8_t opcode, uint8_t address, uint32_t dword);
uint32_t Read_Dword(uint8_t rd_opcode, uint8_t address);
uint32_t Read_Dword_Bits(uint8_t rd_opcode, uint8_t address, uint8_t msbit, uint8_t lsbit);
