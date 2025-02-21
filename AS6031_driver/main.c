#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "AS6031_Coding.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include "SPI_interface.h"

#define TIME_ns(x) (float)(x * 1000000000.0) // result in [ns]
#define INTERRUPT_GPIO_PIN 23

/* USER CODE BEGIN PV */
AS6031_InitTypeDef DUT;        // DUT = Device Under Test
volatile int N_Measure_Cycles; // counter for the while loop
volatile uint8_t My_INTN_State = 1;
volatile float RAW_Result = 0;     // RAW Value in [LSB]
volatile float Time_Result = 0;    // Result in [s]
volatile float Time_Result_ns = 0; // Result in [ns]

// Configuration: using plastic spool piece plastic Audiowell New-Design, V-Shape
uint32_t Reg[20] = {
    0x48DBA399, // [0xC0] CR_WD_DIS
    0x00800401, // [0xC1] CR_IFC_CTRL
    0x00111111, // [0xC2] CR_GP_CTRL
    0x00000001, // [0xC3] CR_USM_OPT
    0x010703FF, // [0xC4] CR_IEH
    0x60060C08, // [0xC5] CR_CPM
    0x01012100, // [0xC6] CR_MRG_TS
    0x00240000, // [0xC7] CR_TPM
    0x00680064, // [0xC8] CR_USM_PRC
    0x60160202, // [0xC9] CR_USM_FRC
    0x000FEA10, // [0xCA] CR_USM_TOF
    0x00A7DE81, // [0xCB] CR_USM_AM
    0x94A0C46C, // [0xCC] CR_TRIM1
    0x401100C4, // [0xCD] CR_TRIM2
    0x00A7400F, // [0xCE] CR_TRIM3
    0x00000001, // [0xD0] SHR_TOF_RATE
    0x00000D80, // [0xD1] SHR_USM_RLS_DLY_U
    0x00000D80, // [0xD2] SHR_USM_RLS_DLY_D
    0x00000041, // [0xDA] SHR_ZCD_FHL_U
    0x00000041  // [0xDB] SHR_ZCD_FHL_D
};

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

void gpio_callback(int gpio, int level, uint32_t tick)
{
    printf("Interrupt detected on GPIO %d! Level: %d, Timestamp: %u\n", gpio, level, tick);
    My_INTN_State = 0;
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

int main()
{
    spi_init();
    configureISR();
    printf("main!\n");
    fflush(stdout);

    AS6031_Init_CFG(&DUT, Reg);
    Write_Opcode(RC_SYS_RST);

    DUT.State = AS6031_STATE_RESET;

    sleep(3); // Datasheet -> Delay = 1ms... BUT at least 3ms are needed _MH

    // Write Configuration (0xC0 - 0xCE, 0xD0 - 0xD2, 0xDA - 0xDB)
    int offset = 0;
    int i, j = 0;

    for (i = 0; i <= 19; i++)
    {
        if (i == 0)
        {
            offset = 0xC0;
            j = 0;
        }
        if (i == 15)
        {
            offset = 0xD0;
            j = 0;
        }
        if (i == 18)
        {
            offset = 0xDA;
            j = 0;
        }
        Write_Dword(RC_RAA_WR, (offset + j), DUT.CR[i]);
        j++;
    }

    // FW Handling Procedures
    // Datasheet Appendix, section 15.7
    // Phase 1: Wait time (dependent on start option)
    // Phase 2: Preparation (common for all procedures)
    // Phase 3: FW Update (different for procedures [A], [B], [C], [D] )
    // Phase 4: FW Retention Check (common for all procedures)
    printf("phase1!\n");
    fflush(stdout);
    // Phase1: Initial Wait Time
    Write_Opcode(RC_MCT_ON);
    DUT.State = AS6031_STATE_RESET;

    sleep(3);

    // Phase 2: Preparation
    Write_Opcode(RC_BM_REQ);
    Write_Dword(RC_RAA_WR, 0xC0, 0x48DBA399);
    Read_Dword(RC_RAA_WR, 0xC0);
    Write_Dword(RC_RAA_WR, 0xCD, 0x40100000);
    Write_Dword(RC_RAA_WR, 0xC6, 0x00001000);
    Write_Opcode(RC_SV_INIT);
    Write_Opcode(RC_MCT_OFF);
    sleep(1);
    Write_Opcode2(RC_MT_REQ, 0x00);
    sleep(1);
    Write_Dword(RC_RAA_WR, 0xDD, 0x00000007);
    Write_Opcode(RC_RF_CLR);
    Write_Dword(RC_RAA_WR, 0xC4, 0x000AF000);
    Write_Opcode(RC_BM_RLS);
    Write_Dword(RC_RAA_WR, 0xDF, 0x50F5B8CA);
    Write_Dword(RC_RAA_WR, 0xDE, 0x00100000);
    printf("while 1!\n");
    fflush(stdout);
    while (Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0)
    {
    };
    Write_Dword(RC_RAA_WR, 0xDE, 0x00080000);
    printf("while 2!\n");
    fflush(stdout);
    while (Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0)
    {
    };

    printf("phase 3!\n");
    fflush(stdout);

    // Phase 3: FW Update
    Read_Dword(RC_RAA_RD, 0xEC);

    // Write FWC
    for (i = 32; i < FWC_Length; i++)
    {
        Write_Byte2(RC_FWC_WR, i, FWC[i]); // Writing FWC, bytewise with two byte address
    }

    // Write FWD
    Write_Dword(RC_RAA_WR_NVRAM, 0x00, 0x0000AB6A); // Writing Firmware Code User, Checksum
    Write_Dword(RC_RAA_WR_NVRAM, 0x01, 0x00000556); // Writing Firmware Data User, Checksum
    Write_Dword(RC_RAA_WR_NVRAM, 0x02, 0x00010000); // Writing FWD_SIMPLE_SCALE (fd16)
    Write_Dword(RC_RAA_WR_NVRAM, 0x03, 0x00000000); // Writing FWD_ZERO_OFFSET
    Write_Dword(RC_RAA_WR_NVRAM, 0x04, 0x051EB852); // Writing FWD_MAX_TOF_DIFF
    Write_Dword(RC_RAA_WR_NVRAM, 0x05, 0xFAE147AE); // Writing FWD_NEG_TOF_DIFF_LIMIT

    Write_Dword(RC_RAA_WR_NVRAM, 0x5B, 0x0000000A); // Writing FWD_R_PULSE_PER_LITER
    Write_Dword(RC_RAA_WR_NVRAM, 0x5C, 0x000003E8); // Writing FWD_R_PULSE_MAX_FLOW

    Write_Dword(RC_RAA_WR_NVRAM, 0x67, 0x00000000); // Writing FWD_USM_RLS_DLY_INIT

    Write_Dword(RC_RAA_WR_NVRAM, 0x6B, 0xABCD7654); // Writing Boot-Loader Release Code

    Write_Dword(RC_RAA_WR, 0xDF, 0x50F5B8CA);
    Write_Dword(RC_RAA_WR, 0xDE, 0x00010000);
    printf("while 3!\n");
    fflush(stdout);
    while (Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0)
    {
    };

    // Phase 4: FW Retention Check
    Write_Dword(RC_RAA_WR, 0xDF, 0x50F5B8CA);
    Write_Dword(RC_RAA_WR, 0xDE, 0x00100000);
    while (Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0)
    {
    };
    printf("while 4!\n");
    fflush(stdout);
    Write_Dword(RC_RAA_WR, 0xDE, 0x00080000);
    while (Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0)
    {
    };
    printf("while 5!\n");
    fflush(stdout);
    Write_Dword(RC_RAA_WR, 0xD3, 0x0007F000);
    sleep(3); // After initialization checksum error flags, sleep of at least 34ms are needed _MH
    Write_Opcode(RC_FW_CHKSUM);
    printf("while 6 !\n");
    fflush(stdout);
    while (Read_Dword_Bits(RC_RAA_RD, 0xE0, 3, 3) == 0)
    {
    };
    Read_Dword(RC_RAA_RD, 0xD3);

    // END
    Write_Opcode(RC_SYS_RST);

    while (1)
    {
        printf("main while\n");
        fflush(stdout);
        /* USER CODE END WHILE */
        sleep(1);
        /* USER CODE BEGIN 3 */
        N_Measure_Cycles++;

        // Wait for INTN
        // NVIC Functionality to increase the speed of MCU
        int timeout = 1000; // Timeout in milliseconds
        while ((My_INTN_State == 1) && (timeout > 0))
        {
            usleep(1000); // Sleep for 1 millisecond
            timeout--;
        }
        if (timeout == 0)
        {
            printf("Timeout occurred while waiting for INTN\n");
            fflush(stdout);
        }
        RAW_Result = (float)Read_Dword(RC_RAA_RD, 0x88); // FDB_US_TOF_0_U
        RAW_Result = (float)Read_Dword(RC_RAA_RD, 0x88); // FDB_US_TOF_0_U

        // Post Processing

        // RAW_Result = Read_Dword(RC_RAA_RD, 0x80);  // FDB_US_TOF_SUM_OF_ALL_U
        // RAW_Result = Read_Dword(RC_RAA_RD, 0x84);  // FDB_US_TOF_SUM_OF_ALL_D
        // RAW_Result /= DUT.Param.CR10.TOF_HIT_SUM_NO;  // Divided by number of hits

        RAW_Result = Read_Dword(RC_RAA_RD, 0x88); // FDB_US_TOF_0_U

        RAW_Result /= 65536.0; // divided by 2^16
        Time_Result = RAW_Result * 250 * (1e-9);

        Time_Result_ns = TIME_ns(Time_Result); // result in [ns]

        printf("Time_Result: %f\n", Time_Result);
        printf("Time_Result_ns: %f\n", Time_Result_ns);
        fflush(stdout);

        // Clear INTN
        Write_Opcode(RC_IF_CLR);
        My_INTN_State = 1;
    }
    /* USER CODE END 3 */
    return 0;
}