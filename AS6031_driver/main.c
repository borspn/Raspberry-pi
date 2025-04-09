#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <getopt.h>
#include <unistd.h>
#include "SPI_interface.h"
#include "user_GP30_parameter.h"
#include "user_UFC_cmd.h"
#include "user_AS6031_parameter.h"

#define STORE_DATA_IN_FILE 0 // 0 = no, 1 = yes
#define MEASURMENT_DELAY_IN_S 2

#define DEFAULT_SPI_SPEED_HZ 500000 // Set SPI clock speed to 500kHz
#define DEFAULT_MEAS_DELAY_IN_S 1   // Default SPI mode
#define DEFAULT_CS_GPIO 25

#define TIME_ns(x) (float)((x) * 1000000000.0) // result in [ns]
#define CHIPNAME "gpiochip0"

#define SPEED_OF_SOUND_WATER 1480.0
#define LEN_OF_SENS 0.06456
#define CROSS_AREA 0.000501653
#define K_FACT 1.008
#define VFR_CONSTANT 15850.32

float velocity = 0;
float volumetricFlowRate = 0;
volatile uint32_t My_ERROR_Counter = 0;

volatile uint8_t My_New_Configuration = 1; // 1 = TDC-GP30 or AS6031 dependent on definition
volatile uint8_t My_New_FHL = 0;
volatile float My_New_FHL_mV = 0;
volatile float My_Set_FHL_mV = 0;

uint32_t MyRAWValueUP;
uint32_t MyRAWValueDOWN;
uint32_t MyRAWAMCVH;
uint32_t MyRAWAMCVL;
uint32_t MyRAWPWUP;
uint32_t MyRAWPWDOWN;

float MyTOFSumAvgUP;
float MyTOFSumAvgDOWN;
float MyDiffTOFSumAvg;
float MyRealAMUP;
float MyRealAMDOWN;
float MyRealPWUP;
float MyRealPWDOWN;
float MyTOFSumAvgUP_ns;
float MyTOFSumAvgDOWN_ns;
float MyDiffTOFSumAvg_ps;

// For Debugging
volatile uint8_t My_Chip_initialized = 0;

volatile uint8_t My_Chip_idle_state = 0;

// Firmware Code: <AS6031_AS6040_A1.F1.11.01_DIF_over_PI_sim.hex>
uint8_t FWC[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xA1, 0xF1, 0x11, 0x01, 0xF2, 0xDC, 0x61, 0x1B, 0x64, 0x16, 0xF2, 0xDC, 0x61, 0x13, 0x64, 0x31,
    0xF2, 0xDC, 0x61, 0x23, 0x64, 0x2E, 0xF2, 0xDC, 0x61, 0x2B, 0x64, 0x76, 0xC9, 0x01, 0x1D, 0xF2,
    0xE2, 0x61, 0x37, 0xC9, 0x01, 0x1D, 0xB8, 0x77, 0xF2, 0x80, 0x2D, 0xF2, 0x84, 0x37, 0x7D, 0xCA,
    0x00, 0x66, 0xF2, 0xC1, 0x61, 0x43, 0xCA, 0xF9, 0x11, 0xF2, 0xDD, 0xF1, 0xAB, 0xC9, 0x01, 0x1D,
    0xC9, 0x01, 0x1D, 0xC9, 0x01, 0x1D, 0xF2, 0x80, 0x77, 0xF2, 0x84, 0x7B, 0x88, 0x73, 0xCB, 0x74,
    0xF2, 0xA5, 0x7D, 0x76, 0x88, 0x73, 0xCB, 0x74, 0xF2, 0xA6, 0x7D, 0xF2, 0xA6, 0x73, 0xF2, 0xA5,
    0x33, 0x87, 0x7C, 0xF3, 0x03, 0x77, 0x34, 0x71, 0xF3, 0x05, 0x0F, 0x4B, 0x13, 0x5D, 0x73, 0xF3,
    0x04, 0x13, 0x4D, 0x73, 0xF2, 0x5E, 0x77, 0xCB, 0x54, 0xCA, 0xFD, 0x67, 0x75, 0x3B, 0x9A, 0xCA,
    0x00, 0xCB, 0x54, 0xCA, 0xFD, 0x67, 0xF3, 0x02, 0x77, 0xCB, 0x54, 0xCA, 0xFD, 0x67, 0x82, 0x7C,
    0xCF, 0xF3, 0x67, 0x73, 0xF2, 0xD1, 0x7C, 0xF2, 0xD2, 0x7C, 0xF2, 0x5F, 0x7F, 0x00, 0x3D, 0x09,
    0x00, 0xF2, 0x5E, 0x7F, 0x00, 0x00, 0x04, 0x31, 0xF2, 0xC5, 0x62, 0x03, 0xF2, 0x5F, 0xD3, 0xF2,
    0xC5, 0x62, 0x03, 0xF2, 0x5E, 0xC3, 0xF2, 0xCA, 0x73, 0xCB, 0x80, 0x00, 0x00, 0x1F, 0x00, 0xCE,
    0x70, 0x88, 0x7C, 0xF2, 0xC6, 0x73, 0xCB, 0x80, 0x00, 0x00, 0x1F, 0xFF, 0x09, 0xCB, 0xA5, 0x00,
    0x00, 0x00, 0x1F, 0xF2, 0xD0, 0x7B, 0xCB, 0x86, 0xCB, 0x54, 0xCE, 0x58, 0xF2, 0xE3, 0x63, 0x47,
    0x75, 0x00, 0x01, 0x06, 0x24, 0xCB, 0x54, 0xCA, 0xFD, 0x67, 0xF2, 0x9E, 0x7C, 0xF3, 0x5B, 0x73,
    0xF3, 0x5C, 0x77, 0xF2, 0x9E, 0x7B, 0xCA, 0xF3, 0x1C, 0x8A, 0x7C, 0x64, 0x01, 0xF2, 0xDC, 0x0B,
    0xCD};

int FWC_Length = sizeof(FWC);

/* @brief  This function takes about ~88�s, using POW() two times! Means,
 *             each function call of POW() takes approx. 40�s AND header
 *             file is needed!
 *                     #include <tgmath.h>
 *                     [..]
 *                     exp = POW(2, bit);
 *                     half_exp = POW(2, bit-1);
 *
 *             ON THE OTHER HAND, using if-clauses, THIS function takes ~2�s
 *                     [..]
 *                     if (bit==16) exp = 65536;
 *                     half_exp = exp / 2;
 *
 *             Definition Two's Complement: Negative numbers in binary
 *                     Given a set of all possible N-bit values, we can assign
 *                     the lower (by the binary value) half to be the integers
 *                     from 0 to (2^N-1 - 1) inclusive and the upper half to
 *                     be (-2N-1) to -1 inclusive
 *
 *             Example:
 *                     // divided by 2^16 (fpp), multipled by 250ns (e.g. T_ref)
 *                     FLOAT_Value = Two_s_Complement_Conversion(HEX_value, 16, 250E-9);
 *
 * @param  raw_number (uint32_t)
 * @param  bit (int)
 * @param  mult_factor (float)
 * @retval Two's Complement (float)
 */
float Two_s_Complement_Conversion(uint32_t raw_number, int bit, float mult_factor)
{
    float number;
    double exp, half_exp;

    /* determine the 'power of 2' */
    if (bit == 32)
        exp = 4294967296; /* = 2^32 */
    if (bit == 24)
        exp = 16777216; /* = 2^24 */
    if (bit == 16)
        exp = 65536; /* = 2^16 */
    if (bit == 8)
        exp = 256; /* = 2^8 */

    half_exp = exp / 2;

    number = raw_number / exp;

    if (number <= (half_exp - 1))
    {
        /*positive number, nothing to do */
    }
    else
    { /**/
        /*to get negative number */
        number -= exp;
    }

    /*to get the correct result by multiplication factor */
    number *= mult_factor;

    return number;
}

/**
 * @brief Configures and writes settings to the AS6031 device.
 *
 * This function sets up the configuration registers and system handling registers
 * for the AS6031 device. It uses predefined configuration values and writes them
 * to the device using specific opcodes and register write functions.
 *
 * @details
 * - The function defines a high-speed clock (HS_CLOCK) of 4 MHz.
 * - Configuration registers (CFG_Registers) are initialized with specific values
 *   for the device's operation, including trimming and timing parameters.
 * - The TOF_HIT_NO value is extracted and processed using a mask and shift operation.
 * - The function writes the configuration registers to the device using
 *   `Write_Register_Auto_Incr`.
 * - System handling registers are written individually using `Write_Dword` for
 *   parameters such as TOF rate, multi-hit start delays, and zero-cross detection levels.
 * - Several opcodes are sent to the device to manage its state, including bus master
 *   request/release and interrupt flag clearing.
 *
 * @note Adjustments to the TRIM2 register value are documented in the comments.
 * @note The function ensures proper sequencing of operations to configure the device.
 *
 * @return None
 */
void writeConfig(void)
{
#undef HS_CLOCK
#define HS_CLOCK 4e6
    // Configuration: using plastic spool piece plastic Audiowell New-Design, V-Shape
    uint32_t CFG_Registers[20] = {
        0x48DBA399, // [0xC0] CR_WD_DIS
        0x00800401, // [0xC1] CR_IFC_CTRL
        0x00000000, // [0xC2] CR_GP_CTRL
        0x00000001, // [0xC3] CR_USM_OPT
        0x0011F7FF, // [0xC4] CR_IEH
        0x6046EF29, // [0xC5] CR_CPM
        0x01012100, // [0xC6] CR_MRG_TS
        0x00240000, // [0xC7] CR_TPM
        0x006807E4, // [0xC8] CR_USM_PRC
        0x60160204, // [0xC9] CR_USM_FRC
        0x010FEA14, // [0xCA] CR_USM_TOF
        0x23A4DE81, // [0xCB] CR_USM_AM
        0x94A0C46C, // [0xCC] CR_TRIM1
        0x401100C4, // [0xCD] CR_TRIM2
        0x00A7400F, // [0xCE] CR_TRIM3
        0x00000001, // [0xD0] SHR_TOF_RATE
        0x000015E0, // [0xD1] SHR_USM_RLS_DLY_U
        0x000015E0, // [0xD2] SHR_USM_RLS_DLY_D
        0x0000004B, // [0xDA] SHR_ZCD_FHL_U
        0x0000004B  // [0xDB] SHR_ZCD_FHL_D
    };
    // TRIM2 adjusted, 0x401100C7 -> 0x401100C4
    // Extracting needed data
    TOF_HIT_NO = CFG_Registers[0xA];
    TOF_HIT_NO &= TOF_HIT_NO_mask;
    TOF_HIT_NO >>= 8;
    Write_Opcode(RC_BM_REQ); // Bus Master Request
    Write_Opcode(RC_MCT_OFF);
    // Configuration Register
    Write_Register_Auto_Incr(RC_RAA_WR_RAM, 0xC0, CFG_Registers, 0xCF);

    // System Handling Register
    Write_Dword(RC_RAA_WR_RAM, SHR_TOF_RATE, 0x00000001);      // TOF RATE Lvl
    Write_Dword(RC_RAA_WR_RAM, SHR_USM_RLS_DLY_U, 0x000015E0); // Multi-hit Start Delay Up
    Write_Dword(RC_RAA_WR_RAM, SHR_USM_RLS_DLY_D, 0x000015E0); // Multi-hit Start Delay Down
    Write_Dword(RC_RAA_WR_RAM, SHR_ZCD_FHL_U, 0x0000004B);     // Zero Cross Detection First Hit Level Up
    //    Write_Dword(RC_RAA_WR_RAM, SHR_ZCD_FHL_D,         0x00000087);  //Zero Cross Detection First Hit Level Down
    Write_Dword(RC_RAA_WR_RAM, SHR_ZCD_FHL_D, 0x0000004B); // Zero Cross Detection First Hit Level Down
    Write_Opcode(RC_MCT_ON);
    Write_Opcode(RC_IF_CLR);
    Write_Opcode(RC_BM_RLS); // Bus Master Release

    return;
}

/**
 * @brief Calculates the Time of Flight (TOF) based on a given memory address.
 *
 * This function reads a raw 32-bit value from the specified TOF address in memory,
 * performs a two's complement conversion, and calculates the corresponding
 * floating-point Time of Flight value.
 *
 * @param TOF_address The memory address from which the raw TOF value is read.
 * @return The calculated Time of Flight as a floating-point value.
 *
 * @note The function relies on the `Read_Dword` function to fetch the raw value
 *       and the `Two_s_Complement_Conversion` function to perform the conversion.
 * @note The `T_REF` constant is used during the two's complement conversion.
 */
float Calc_TimeOfFlight(uint32_t TOF_address)
{
    /* local parameter */
    uint32_t RAWValue = 0;
    float FLOATValue = 0;

    RAWValue = Read_Dword(RC_RAA_RD_RAM, TOF_address);
    /* Calculation of Time of Flight */
    FLOATValue = Two_s_Complement_Conversion(RAWValue, 16, T_REF);

    return FLOATValue;
}

/**
 * @brief Initializes the state variables for the system.
 *
 * This function resets the error counter, clears any new configuration flags,
 * and marks the chip as initialized. It is typically called during the
 * initialization phase of the system to ensure all relevant state variables
 * are set to their default values.
 *
 * @note Ensure this function is called before any other operations that
 * depend on the initialized state of the chip.
 */
void My_Init_State(void)
{
    My_ERROR_Counter = 0;
    My_New_Configuration = 0;
    My_Chip_initialized = 1;
}

/**
 * @brief Calculates the amplitude based on raw data and calibration parameters.
 *
 * This function reads a raw value from a specified memory address and calculates
 * the amplitude using the provided calibration parameters (AMC_VH and AMC_VL).
 *
 * @param AM_address The memory address to read the raw value from.
 * @param AMC_VH The high calibration value.
 * @param AMC_VL The low calibration value.
 * @return The calculated amplitude as a floating-point value.
 */
float Calc_Amplitude(uint32_t AM_address, uint32_t AMC_VH, uint32_t AMC_VL)
{
    /* local parameter */
    uint32_t RAWValue = 0;
    float AMC_gradient = 0;
    float AMC_offset = 0;
    float FLOATValue = 0;

    RAWValue = Read_Dword(RC_RAA_RD_RAM, AM_address);
    AMC_gradient = 350 / (float)(AMC_VH - AMC_VL);
    AMC_offset = ((2 * AMC_VL) - AMC_VH) * AMC_gradient;

    FLOATValue = (AMC_gradient * RAWValue) - AMC_offset;

    return FLOATValue;
}

/**
 * @brief Processes the Time of Flight (TOF) data and performs error handling,
 *        calibration, and data post-processing.
 *
 * This function reads and processes TOF data from a sensor, handles errors,
 * updates calibration values, calculates TOF and amplitude values, and optionally
 * stores the processed data in a file. It also clears interrupt and error flags
 * after processing.
 *
 * @note The function behavior can be modified by defining the macro `STORE_DATA_IN_FILE`.
 *       When defined, the processed data is appended to a file named "data.csv".
 *
 * Steps performed:
 * 1. Reads the SRR_ERR_FLAG to check for errors in the last measurement cycle.
 * 2. Handles errors by clearing error flags and optionally reinitializing the chip.
 * 3. Updates amplitude calibration values if required.
 * 4. Calculates amplitude and TOF values.
 * 5. Performs post-processing, including scaling and calculating differences.
 * 6. Optionally writes the processed data to a file.
 * 7. Clears interrupt, error, and frontend status flags.
 *
 * @global_variables
 * - `SRR_ERR_FLAG_content`: Stores the error flag content.
 * - `My_ERROR_Counter`: Tracks the number of errors encountered.
 * - `MyRAWAMCVH`, `MyRAWAMCVL`: Raw amplitude calibration values.
 * - `MyRealAMUP`, `MyRealAMDOWN`: Calculated amplitude values.
 * - `MyTOFSumAvgUP`, `MyTOFSumAvgDOWN`: Average TOF values.
 * - `MyDiffTOFSumAvg`: Difference between TOF averages.
 * - `MyRAWPWUP`, `MyRAWPWDOWN`: Raw pulse width values.
 * - `MyRealPWUP`, `MyRealPWDOWN`: Scaled pulse width values.
 * - `MyTOFSumAvgUP_ns`, `MyTOFSumAvgDOWN_ns`, `MyDiffTOFSumAvg_ps`: Scaled TOF values.
 *
 * @dependencies
 * - `Read_Dword()`: Reads a 32-bit value from the sensor.
 * - `Write_Dword()`: Writes a 32-bit value to the sensor.
 * - `Calc_Amplitude()`: Calculates amplitude based on raw values.
 * - `Calc_TimeOfFlight()`: Calculates TOF based on raw values.
 *
 * @file_output
 * - File: "data.csv" (if `STORE_DATA_IN_FILE` is defined)
 * - Format: Tab-separated values with columns:
 *   - `MyTOFSumAvgUP_ns`
 *   - `MyTOFSumAvgDOWN_ns`
 *   - `MyDiffTOFSumAvg_ps`
 *
 * @error_handling
 * - If an error is detected in `SRR_ERR_FLAG`, the error is logged, and flags are cleared.
 * - If the file "data.csv" cannot be opened, an error message is printed, and the function returns.
 *
 * @clears_flags
 * - Clears interrupt, error, and frontend status flags by writing to `SHR_EXC`.
 */
void Process_TOF(void)
{
    printf("Process_TOF\n");
    fflush(stdout);
#ifdef STORE_DATA_IN_FILE
    FILE *file = fopen("data.csv", "a");
    if (file == NULL)
    {
        perror("Error opening file");
        return;
    }

    // Write header if file is empty
    fseek(file, 0, SEEK_END);
    if (ftell(file) == 0)
    {
        fprintf(file, "MyTOFSumAvgUP_ns\tMyTOFSumAvgDOWN_ns\tMyDiffTOFSumAvg_ps\n");
    }
#endif // STORE_DATA_IN_FILE
    /* Time Conversion Mode */
    /* Cylce A = ~ 370 µs @SPI = 2.5 MHz*/
    /* Cylce B = ~ 160 µs @SPI = 2.5 MHz*/

    /* STEP - Read SRR_ERR_FLAG to check if any error
     *  occurred during last measurement cycle */
    SRR_ERR_FLAG_content = Read_Dword(RC_RAA_RD_RAM, SRR_ERR_FLAG);

    if (SRR_ERR_FLAG_content > 0)
    {
        printf("SRR_ERR_FLAG_content%d\n", SRR_ERR_FLAG_content);
        fflush(stdout);
        // Error Handling with simplified query
        printf("...error!\n");
        fflush(stdout);
        My_ERROR_Counter++;

        /* Chip has to be reinitialized */
        // My_Chip_initialized = 0;
        // My_Chip_idle_state = 0;
        // Skipping Post Processing
        Write_Dword(RC_RAA_WR_RAM, SHR_EXC, (FES_CLR_mask | EF_CLR_mask));
        SRR_ERR_FLAG_content = Read_Dword(RC_RAA_RD_RAM, SRR_ERR_FLAG);
    }
    else
    {
        if (SRR_FEP_STF_content & (US_AM_UPD_mask))
        {
            /* If amplitude calibration values = ZERO
             * Reloading amplitude calibration values */
            if (MyRAWAMCVH == 0 || MyRAWAMCVL == 0)
            {
                MyRAWAMCVH = Read_Dword(RC_RAA_RD_RAM, FDB_US_AMC_VH);
                MyRAWAMCVL = Read_Dword(RC_RAA_RD_RAM, FDB_US_AMC_VL);
                printf("MyRAWAMCVH%d MyRAWAMCVL%d\n", MyRAWAMCVH, MyRAWAMCVL);
                fflush(stdout);
            }

            /* If amplitude calibration values are available
             * Updating amplitude values */
            if (MyRAWAMCVH != 0 && MyRAWAMCVL != 0)
            {
                MyRealAMUP = Calc_Amplitude(FDB_US_AM_U, MyRAWAMCVH, MyRAWAMCVL);
                MyRealAMDOWN = Calc_Amplitude(FDB_US_AM_D, MyRAWAMCVH, MyRAWAMCVL);
                printf("MyRealAMUP%f MyRealAMDOWN%f\n", MyRealAMUP, MyRealAMDOWN);
                fflush(stdout);
            }
        }

        /* Updating TOF Values */
        MyTOFSumAvgUP = Calc_TimeOfFlight(FDB_US_TOF_ADD_ALL_U) / TOF_HIT_NO;
        MyTOFSumAvgDOWN = Calc_TimeOfFlight(FDB_US_TOF_ADD_ALL_D) / TOF_HIT_NO;

        /* Updating Pulse Width Ratio */
        MyRAWPWUP = Read_Dword(RC_RAA_RD_RAM, FDB_US_PW_U);
        MyRAWPWDOWN = Read_Dword(RC_RAA_RD_RAM, FDB_US_PW_D);

        // post processing and calculation
        MyDiffTOFSumAvg = (MyTOFSumAvgDOWN - MyTOFSumAvgUP);

        MyRealPWUP = MyRAWPWUP;
        MyRealPWUP /= (1 << 7);
        MyRealPWDOWN = MyRAWPWDOWN;
        MyRealPWDOWN /= (1 << 7);

        // scaling
        MyTOFSumAvgUP_ns = MyTOFSumAvgUP / 1e-9;
        MyTOFSumAvgDOWN_ns = MyTOFSumAvgDOWN / 1e-9;
        MyDiffTOFSumAvg_ps = MyDiffTOFSumAvg / 1e-12;

        velocity = (abs(MyDiffTOFSumAvg) * (SPEED_OF_SOUND_WATER * SPEED_OF_SOUND_WATER)) / (2 * LEN_OF_SENS);
        printf("abs(MyDiffTOFSumAvg) = %f\n", MyDiffTOFSumAvg);
        printf("SPEED_OF_SOUND_WATER = %f\n", SPEED_OF_SOUND_WATER);
        printf("LEN_OF_SENS = %f\n", LEN_OF_SENS);
        fflush(stdout);
        volumetricFlowRate = VFR_CONSTANT * K_FACT * velocity * CROSS_AREA;

        printf("TOF data: %.3f\t%.3f\t%.3f\n", MyTOFSumAvgUP_ns, MyTOFSumAvgDOWN_ns, MyDiffTOFSumAvg_ps);
        printf("Velocity: %f\n", velocity);
        printf("Volumetric Flow Rate: %f\n", volumetricFlowRate);
        fflush(stdout);

#ifdef STORE_DATA_IN_FILE
        fprintf(file, "%.3f\t%.3f\t%.3f\n", MyTOFSumAvgUP_ns, MyTOFSumAvgDOWN_ns, MyDiffTOFSumAvg_ps);
        fclose(file);
#endif // STORE_DATA_IN_FILE
    }
    /* Clear interrupt flag, error flag & frontend status
     * flag register by writing code to SHR_EXC */
    Write_Dword(RC_RAA_WR_RAM, SHR_EXC, (FES_CLR_mask | EF_CLR_mask | IF_CLR_mask));
}

int main(int argc, char *argv[])
{
    printf("main!\n");
    fflush(stdout);

    uint32_t spiSpeed = DEFAULT_SPI_SPEED_HZ;
    uint8_t csGpio = DEFAULT_CS_GPIO;
    float measDelay = DEFAULT_MEAS_DELAY_IN_S;

    int opt;
    while ((opt = getopt(argc, argv, "s:c:d:")) != -1)
    {
        switch (opt)
        {
        case 's':
            spiSpeed = atoi(optarg);
            if (spiSpeed <= 0)
            {
                fprintf(stderr, "Invalid SPI speed. Using default: %d Hz\n", DEFAULT_SPI_SPEED_HZ);
                spiSpeed = DEFAULT_SPI_SPEED_HZ;
            }
            break;
        case 'c':
            csGpio = atoi(optarg);
            if (csGpio < 0)
            {
                fprintf(stderr, "Invalid CS GPIO. Using default: %d\n", DEFAULT_CS_GPIO);
                csGpio = DEFAULT_CS_GPIO;
            }
            break;
        case 'd':
            measDelay = atof(optarg);
            if (measDelay <= 0)
            {
                fprintf(stderr, "Invalid measurement delay. Using default: %.2f seconds\n", (float)DEFAULT_MEAS_DELAY_IN_S);
                measDelay = DEFAULT_MEAS_DELAY_IN_S;
            }
            break;
        default:
            fprintf(stderr, "Usage: %s [-s spiSpeed] [-d measDelay]\n", argv[0]);
            exit(EXIT_FAILURE);
        }
    }

    printf("Starting with SPI speed: %d Hz, CS GPIO: %d , Measurement delay: %.2f seconds\n", spiSpeed, csGpio, measDelay);

    spi_init(csGpio, spiSpeed);
    Write_Dword(RC_RAA_WR_RAM, SHR_EXC, (FES_CLR_mask | EF_CLR_mask | IF_CLR_mask));

    while (1)
    {
        printf("while main!\n");
        fflush(stdout);

        sleep(measDelay);

        if ((My_Chip_initialized == 1))
        {
            printf("(My_Chip_initialized == 1)\n");
            fflush(stdout);

            Process_TOF();

            /* Update Configuration */
            if (My_New_FHL_mV > 0)
            {
                printf("My_New_FHL_mV > 0\n");
                fflush(stdout);
                /* Update System Handling Register
                 * SHR_FHL_U (First Hit Level Up) 0x0DA
                 * SHR_FHL_D (First Hit Level Down) 0x0DB
                 */
                My_New_FHL = My_New_FHL_mV / 0.88;

                Write_Dword(RC_RAA_WR_RAM, SHR_FHL_U, My_New_FHL);
                Write_Dword(RC_RAA_WR_RAM, SHR_FHL_D, My_New_FHL);

                My_Set_FHL_mV = My_New_FHL_mV;
                My_New_FHL_mV = 0;
            }
        }

        /* Reload Configuration */
        if (My_New_Configuration > 0)
        {
            printf("My_New_Configuration > 0\n");
            fflush(stdout);
            My_Chip_initialized = 0;

            // Put_UFC_Into_Idle();
            My_Chip_idle_state = 1;

            if (My_New_Configuration != 99)
            {
                printf("My_New_Configuration != 99\n");
                fflush(stdout);

                writeConfig(); // Write Configuration to AS6031

                My_Set_FHL_mV = Read_Dword(RC_RAA_RD_RAM, SHR_ZCD_FHL_U);
                My_Set_FHL_mV *= 0.88;

                Write_Opcode(RC_MCT_ON); // RC_MCT_ON
                // Write_Opcode(RC_IF_CLR);
                My_Chip_idle_state = 0;
                My_Init_State();
            }
        }
    } // End of while loop
}
