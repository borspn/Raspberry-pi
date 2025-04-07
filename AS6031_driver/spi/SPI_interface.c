#include "SPI_interface.h"
#include <linux/spi/spidev.h>  // For SPI_IOC_WR_MAX_SPEED_HZ
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


#define SPI_SPEED_HZ 500000  // Set SPI clock speed to 500kHz

int spi_handle;
struct gpiod_chip *chip;
struct gpiod_line *cs_line;

/**
 * @brief Initializes the SPI interface and configures the necessary GPIO and SPI settings.
 *
 * This function performs the following steps:
 * 1. Opens the GPIO chip using libgpiod.
 * 2. Retrieves the GPIO line for the Chip Select (CS) pin and configures it as an output.
 * 3. Opens the SPI device file (e.g., /dev/spidev0.0).
 * 4. Configures the SPI mode and speed using ioctl system calls.
 *
 * If any step fails, the function prints an error message to stderr and terminates the program.
 *
 * @note The CS GPIO pin is set to HIGH (inactive) by default after initialization.
 * @note The SPI mode is set to SPI_MODE_1 by default. Modify the `mode` variable if a different mode is required.
 * @note The SPI speed is set using the `SPI_SPEED_HZ` macro. Ensure this macro is defined with the desired speed.
 *
 * @warning This function exits the program on failure. Ensure proper error handling if integrating into a larger system.
 *
 * @dependencies
 * - Requires the libgpiod library for GPIO operations.
 * - Requires access to the SPI device file (e.g., /dev/spidev0.0).**/
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


    uint8_t mode = SPI_MODE_1;  // Change to SPI_MODE_2 if needed
    if (ioctl(spi_handle, SPI_IOC_WR_MODE, &mode) < 0)
    {
        perror("Failed to set SPI mode");
        close(spi_handle);
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
 * @brief Sends a single-byte opcode to the SPI device.
 *
 * This function writes a single byte (opcode) to the SPI device by
 * lowering the Slave Select (SSN) line, transmitting the byte, and
 * then raising the SSN line to complete the transaction.
 *
 * @param one_byte The opcode to be sent to the SPI device.
 *
 * @note Ensure that the SPI interface is properly initialized before
 *       calling this function. The macros PUT_SSN_LOW and PUT_SSN_HIGH
 *       must be defined to control the SSN line.
 */
void Write_Opcode(char one_byte)
{
    PUT_SSN_LOW;
    write(spi_handle, &one_byte, 1); // Send opcode
    PUT_SSN_HIGH;
}


/**
 * @brief Writes a 32-bit data word to a specified address using SPI communication.
 *
 * This function constructs a 6-byte SPI transmission buffer consisting of an opcode,
 * an address, and a 32-bit data word. It then transmits the buffer over SPI while
 * toggling the Slave Select (SSN) line to indicate the start and end of the transaction.
 *
 * @param opcode The operation code to be sent as the first byte of the SPI transmission.
 * @param address The address to which the data word will be written.
 * @param dword The 32-bit data word to be transmitted.
 *
 * @note The function assumes that the SPI handle (`spi_handle`) and macros for controlling
 *       the SSN line (`PUT_SSN_LOW` and `PUT_SSN_HIGH`) are properly defined and initialized.
 *       The `write` function is used to transmit the data over SPI.
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
    write(spi_handle, spiTX, 6);
    PUT_SSN_HIGH;
}

/**
 * @brief Reads a 32-bit value from a device using SPI communication.
 *
 * This function sends a read opcode and an address to the SPI device, 
 * then reads a 4-byte response and combines it into a 32-bit unsigned integer.
 *
 * @param rd_opcode The read opcode to be sent to the SPI device.
 * @param address The address from which to read data.
 * @return The 32-bit unsigned integer value read from the SPI device.
 *
 * @note This function assumes that the SPI device is configured and 
 *       the `spi_handle` is properly initialized. It also assumes 
 *       that the macros `PUT_SSN_LOW` and `PUT_SSN_HIGH` are defined 
 *       to control the Slave Select (SSN) line.
 */
uint32_t Read_Dword(char rd_opcode, uint8_t address)
{
    uint8_t spiTX[2];
    uint8_t spiRX[4];
    uint32_t temp_u32 = 0;

    spiTX[0] = rd_opcode;
    spiTX[1] = address;
    PUT_SSN_LOW;
    write(spi_handle, spiTX, 2); // Send opcode/address, receive data
    read(spi_handle, spiRX, 4);
    PUT_SSN_HIGH;
    temp_u32 = (spiRX[0] << 24) + (spiRX[1] << 16) + (spiRX[2] << 8) + (spiRX[3]);

    return temp_u32;
}


/**
 * @brief Sends two bytes via SPI interface.
 *
 * This function transmits two bytes over the SPI bus. It ensures that the 
 * Slave Select (SSN) line is properly toggled to indicate the start and 
 * end of the SPI communication.
 *
 * @param byte1 The first byte to be transmitted.
 * @param byte2 The second byte to be transmitted.
 *
 * @note The function assumes that the SPI handle (spi_handle) is already 
 *       initialized and that the macros PUT_SSN_LOW and PUT_SSN_HIGH 
 *       correctly control the SSN line.
 */
void Write_Opcode2(char byte1, char byte2)
{
    char spiTX[2] = {byte1, byte2};
    PUT_SSN_LOW;
    write(spi_handle, spiTX, 2);
    PUT_SSN_HIGH;
}

/**
 * @brief Writes a sequence of 32-bit values to consecutive registers using SPI with auto-increment.
 *
 * This function sends an opcode and a starting register address, followed by a sequence of 32-bit
 * values to be written to consecutive registers. The SPI chip select signal is managed within the
 * function to ensure proper communication.
 *
 * @param opcode      The opcode to be sent as the first byte of the SPI transaction.
 * @param from_addr   The starting register address to write to.
 * @param dword_array Pointer to an array of 32-bit values to be written to the registers.
 * @param to_addr     The ending register address (inclusive) to write to.
 *
 * @note The function assumes that the SPI handle (spi_handle) and the macros PUT_SSN_LOW and
 *       PUT_SSN_HIGH are properly defined and configured elsewhere in the code.
 * @note The caller must ensure that the dword_array contains enough elements to cover the range
 *       from `from_addr` to `to_addr`.
 */

void Write_Register_Auto_Incr(uint8_t opcode, uint8_t from_addr, uint32_t *dword_array, int to_addr)
{
    uint8_t spiTX[4];
    uint32_t temp_u32 = 0;

    spiTX[0] = opcode;
    spiTX[1] = from_addr;

    PUT_SSN_LOW;
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
    PUT_SSN_HIGH;
}