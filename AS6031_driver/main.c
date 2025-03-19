#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include "SPI_interface.h"
#include "user_GP30_parameter.h"
#include "user_UFC_cmd.h"
#include "user_AS6031_parameter.h"

#define TIME_ns(x) (float)((x) * 1000000000.0) // result in [ns]
#define INTERRUPT_GPIO_PIN 23
#define CHIPNAME "gpiochip0"

float velocity = 0;
float speedOfSoundInWaterMps = 1480.0;
float lenSensorInM = 0.06456;

// volatile bool My_INTN_State = false;
volatile uint8_t My_INTN_State = 1; /* low active */

// *** debug - for watchdog reading
volatile uint32_t watchdog_value = 0;

volatile uint32_t My_INTN_Counter = 0;
volatile uint32_t My_Cycle_A_Counter = 0;
volatile uint32_t My_Cycle_B_Counter = 0;
volatile uint32_t My_Loop_Pass_Counter = 0;

volatile uint32_t My_ERROR_Counter = 0;
volatile uint32_t My_UP_zero = 0;
volatile uint32_t My_DOWN_zero = 0;

volatile uint32_t My_Min_Value_A = 0xFFFFFFFF, My_Max_Value_A = 0,
                  My_Min_Value_B = 0xFFFFFFFF, My_Max_Value_B = 0;
volatile uint32_t My_Too_Less_Time = 0;

volatile uint8_t My_New_Configuration = 1; // 1 = TDC-GP30 or AS6031 dependent on definition
volatile uint8_t My_New_FHL = 0;
volatile float My_New_FHL_mV = 0;
volatile float My_Set_FHL_mV = 0;

volatile uint8_t MyMode = 1; /* default */

// Post Processing - Time Conversion Mode (MyMode = 1)
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
float MyRealHSClk;

// Scaling
float MyRealHSClk_ns;
float MyTOFSumAvgUP_ns;
float MyTOFSumAvgDOWN_ns;
float MyDiffTOFSumAvg_ps;

// For Debugging
volatile uint8_t My_Chip_initialized = 0;
volatile uint8_t My_Chip_config_1 = 0;
volatile uint8_t My_Chip_config_2 = 0;
volatile uint8_t My_Chip_config_3 = 0;
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

    // Configuration Register
    Write_Register_Auto_Incr(RC_RAA_WR_RAM, 0xC0, CFG_Registers, 0xCF);

    // System Handling Register
    Write_Dword(RC_RAA_WR_RAM, SHR_TOF_RATE, 0x00000001);      // TOF RATE Lvl
    Write_Dword(RC_RAA_WR_RAM, SHR_USM_RLS_DLY_U, 0x000015E0); // Multi-hit Start Delay Up
    Write_Dword(RC_RAA_WR_RAM, SHR_USM_RLS_DLY_D, 0x000015E0); // Multi-hit Start Delay Down
    Write_Dword(RC_RAA_WR_RAM, SHR_ZCD_FHL_U, 0x0000004B);     // Zero Cross Detection First Hit Level Up
    //    Write_Dword(RC_RAA_WR_RAM, SHR_ZCD_FHL_D,         0x00000087);  //Zero Cross Detection First Hit Level Down
    Write_Dword(RC_RAA_WR_RAM, SHR_ZCD_FHL_D, 0x0000004B); // Zero Cross Detection First Hit Level Down

    return;
}

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

void gpio_callback(int gpio, int level, uint32_t tick)
{
    printf("Interrupt detected on GPIO %d! Level: %d, Timestamp: %u\n", gpio, level, tick);
    if (1)
    {
        My_INTN_State = 0;
    }
}

void My_Init_State(void)
{
    /* reset counter */
    My_INTN_Counter = 0;
    My_Cycle_A_Counter = 0;
    My_Cycle_B_Counter = 0;
    My_Loop_Pass_Counter = 0;
    My_ERROR_Counter = 0;
    My_UP_zero = 0;
    My_DOWN_zero = 0;
    My_Too_Less_Time = 0;

    /* expand range */
    My_Min_Value_A = 0xFFFFFFFF;
    My_Max_Value_A = 0;
    My_Min_Value_B = 0xFFFFFFFF;
    My_Max_Value_B = 0;

    My_New_Configuration = 0;

    My_Chip_initialized = 1;
}

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

bool configureISR()
{
    gpioSetMode(INTERRUPT_GPIO_PIN, PI_INPUT);
    gpioSetPullUpDown(INTERRUPT_GPIO_PIN, PI_PUD_UP);
    if (gpioSetAlertFunc(INTERRUPT_GPIO_PIN, gpio_callback) < 0)
    {
        printf("Failed to set alert function!\n");
        return false;
    }
    return true;
}

void Put_UFC_Into_Idle(void)
{
    Write_Opcode(RC_SYS_RST);                           // Reset UFC completely
    usleep(10000);                                      // delay = 20ms?? only firmware data has no configuration data
    Write_Dword(RC_RAA_WR_RAM, CR_WD_DIS, WD_DIS_CODE); // STEP 1 - Disable Watchdog by writing code to CR_WD_DIS
    Write_Opcode(RC_MCT_OFF);
}

void My_Time_Conversion_Mode(void)
{
    printf("My_Time_Conversion_Mode\n");
    fflush(stdout);

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
    /* Time Conversion Mode */
    /* Cylce A = ~ 370 µs @SPI = 2.5 MHz*/
    /* Cylce B = ~ 160 µs @SPI = 2.5 MHz*/

    /* STEP - Read SRR_ERR_FLAG to check if any error
     *  occurred during last measurement cycle */
    SRR_ERR_FLAG_content = Read_Dword(RC_RAA_RD_RAM, SRR_ERR_FLAG);

    if (SRR_ERR_FLAG_content > 0)
    {
        printf("SRR_ERR_FLAG_content > 0\n");
        fflush(stdout);
        // Error Handling with simplified query
        printf("...error!\n");
        fflush(stdout);
        My_ERROR_Counter++;

        /* Chip has to be reinitialized */
        My_Chip_initialized = 0;
        My_Chip_idle_state = 0;
        /* Skipping Post Processing */
    }
    else
    {
        printf("!SRR_ERR_FLAG_content > 0\n");
        fflush(stdout);

        /* STEP - read the measurement results
         * out of the frontend data buffer */

        My_Cycle_A_Counter += 1; // counts every call
        /* Updating TOF Values */
        MyTOFSumAvgUP = Calc_TimeOfFlight(FDB_US_TOF_ADD_ALL_U) / TOF_HIT_NO;
        MyTOFSumAvgDOWN = Calc_TimeOfFlight(FDB_US_TOF_ADD_ALL_D) / TOF_HIT_NO;
        printf("MyTOFSumAvgUP%f MyTOFSumAvgDOWN%f\n", MyTOFSumAvgUP, MyTOFSumAvgDOWN);
        fflush(stdout);

        // post processing and calculation
        MyDiffTOFSumAvg = (MyTOFSumAvgDOWN - MyTOFSumAvgUP);
        printf("MyDiffTOFSumAvg%f\n", MyDiffTOFSumAvg);
        fflush(stdout);

        MyRealPWUP = MyRAWPWUP;
        MyRealPWUP /= (1 << 7);
        MyRealPWDOWN = MyRAWPWDOWN;
        MyRealPWDOWN /= (1 << 7);

        // scaling
        MyTOFSumAvgUP_ns = MyTOFSumAvgUP / 1e-9;
        MyTOFSumAvgDOWN_ns = MyTOFSumAvgDOWN / 1e-9;
        MyDiffTOFSumAvg_ps = MyDiffTOFSumAvg / 1e-12;

        velocity = ((abs(MyDiffTOFSumAvg_ps) * 1E-9) * (speedOfSoundInWaterMps * speedOfSoundInWaterMps)) / (2 * lenSensorInM);
        printf("velocity%f\n", velocity);
        fflush(stdout);
        fprintf(file, "%.3f\t%.3f\t%.3f\n", MyTOFSumAvgUP_ns, MyTOFSumAvgDOWN_ns, MyDiffTOFSumAvg_ps);
        printf("Appended: %.3f\t%.3f\t%.3f\n", MyTOFSumAvgUP_ns, MyTOFSumAvgDOWN_ns, MyDiffTOFSumAvg_ps);
        fflush(stdout);
    }

    /* STEP 5 - Clear interrupt flag, error flag & frontend status
     * flag register by writing code to SHR_EXC */
    Write_Dword(RC_RAA_WR_RAM, SHR_EXC, (FES_CLR_mask | EF_CLR_mask | IF_CLR_mask));
    My_INTN_State = 1;

} // End of Post Processing

int main()
{
    printf("main!\n");
    fflush(stdout);

    spi_init();
    Write_Dword(RC_RAA_WR_RAM, SHR_EXC, (FES_CLR_mask | EF_CLR_mask | IF_CLR_mask));
    configureISR();
    while (1)
    {
        printf("while main!\n");
        fflush(stdout);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        My_Loop_Pass_Counter += 1; // counts every loop

        //	  printf("%02d:%02d:%02d\n", currTime.Hours, currTime.Minutes, currTime.Seconds);
        printf("My_INTN_State%d\n", My_INTN_State);
        printf("My_Chip_initialized%d\n", My_Chip_initialized);
        fflush(stdout);
        if ((My_INTN_State == 1) && (My_Chip_initialized == 1))
        {
            printf("(My_INTN_State == 0) && (My_Chip_initialized == 1)\n");
            fflush(stdout);

            My_Time_Conversion_Mode();

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
        } // End of (My_INTN_State == true) query

        /* Reload Configuration */
        if (My_New_Configuration > 0)
        {
            printf("My_New_Configuration > 0\n");
            fflush(stdout);
            My_Chip_initialized = 0;
            My_Chip_config_1 = 0;
            My_Chip_config_2 = 0;
            My_Chip_config_3 = 0;

            Put_UFC_Into_Idle();
            My_Chip_idle_state = 1;

            if (My_New_Configuration != 99)
            {
                printf("My_New_Configuration != 99\n");
                fflush(stdout);
                // AS6031_ST_NS
                // strcpy(My_Configuration, "AS6031_ST_NS");
                My_Chip_config_2 = 1;

                writeConfig();

                My_Set_FHL_mV = Read_Dword(RC_RAA_RD_RAM, SHR_ZCD_FHL_U);
                My_Set_FHL_mV *= 0.88;

                Write_Opcode(RC_MCT_ON); // RC_MCT_ON
                // Write_Opcode(RC_IF_CLR);
                My_Chip_idle_state = 0;
                My_Init_State();
            }
        }

        // With any ERROR
        // Initialisation of DUT will be cleared
        if (Read_Dword(RC_RAA_RD_RAM, SRR_ERR_FLAG))
        {
            printf("Read_Dword(RC_RAA_RD_RAM, SRR_ERR_FLAG)\n");
            fflush(stdout);
            My_Chip_initialized = 0;
            My_Init_State();
        }

    } // End of while loop
}
