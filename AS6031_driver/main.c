#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "AS6031_Coding.h"
#include <stdint.h>
#include "SPI_interface.h"

/* USER CODE BEGIN PV */
AS6031_InitTypeDef DUT;        // DUT = Device Under Test
volatile int N_Measure_Cycles; // counter for the while loop
volatile uint8_t   My_INTN_State = 1;
volatile float     RAW_Result = 0;                 // RAW Value in [LSB]
volatile float     Time_Result = 0;                // Result in [s]
volatile float     Time_Result_ns = 0;             // Result in [ns]

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

void setup()
{
    // Initialize wiringPi and SPI
    wiringPiSetupGpio();
    if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1)
    {
        // Handle error
        return;
    }

    // Set SSN pin as output
    pinMode(GPIO_PIN_SSN, OUTPUT);
    digitalWrite(GPIO_PIN_SSN, HIGH); // Deselect the sensor
}

int main()
{
    setup();

    AS6031_Init_CFG(&DUT, Reg);
    Write_Opcode(RC_SYS_RST);

    DUT.State = AS6031_STATE_RESET;

    delay(3); // Datasheet -> Delay = 1ms... BUT at least 3ms are needed _MH

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

    // Phase1: Initial Wait Time
    Write_Opcode(RC_MCT_ON);
    DUT.State = AS6031_STATE_RESET;

    delay(3);

    // Phase 2: Preparation
    Write_Opcode(RC_BM_REQ);
    Write_Dword(RC_RAA_WR, 0xC0, 0x48DBA399);
    Write_Dword(RC_RAA_WR, 0xCD, 0x40100000);
    Write_Dword(RC_RAA_WR, 0xC6, 0x00001000);
    Write_Opcode(RC_SV_INIT);
    Write_Opcode(RC_MCT_OFF);
    delay(1);
    Write_Opcode2(RC_MT_REQ, 0x00);
    delay(1);
    Write_Dword(RC_RAA_WR, 0xDD, 0x00000007);
    Write_Opcode(RC_RF_CLR);
    Write_Dword(RC_RAA_WR, 0xC4, 0x000AF000);
    Write_Opcode(RC_BM_RLS);
    Write_Dword(RC_RAA_WR, 0xDF, 0x50F5B8CA);
    Write_Dword(RC_RAA_WR, 0xDE, 0x00100000);
    while (Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0)
    {
    };
    Write_Dword(RC_RAA_WR, 0xDE, 0x00080000);
    while (Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0)
    {
    };

    // Phase 3: FW Update
    Read_Dword(RC_RAA_RD, 0xEC);

    // Write FWC
    for (i = 32; i <= FWC_Length; i++)
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

    while (Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0)
    {
    };

    // Phase 4: FW Retention Check
    Write_Dword(RC_RAA_WR, 0xDF, 0x50F5B8CA);
    Write_Dword(RC_RAA_WR, 0xDE, 0x00100000);
    while (Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0)
    {
    };
    Write_Dword(RC_RAA_WR, 0xDE, 0x00080000);
    while (Read_Dword_Bits(RC_RAA_RD, 0xE0, 1, 1) == 0)
    {
    };
    Write_Dword(RC_RAA_WR, 0xD3, 0x0007F000);
    delay(3); // After initialization checksum error flags, delay of at least 34ms are needed _MH
    Write_Opcode(RC_FW_CHKSUM);
    while (Read_Dword_Bits(RC_RAA_RD, 0xE0, 3, 3) == 0)
    {
    };
    Read_Dword(RC_RAA_RD, 0xD3);

    // END
    Write_Opcode(RC_SYS_RST);

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        N_Measure_Cycles++;

        // Wait for INTN
        // NVIC Functionality to increase the speed of MCU
        while ((My_INTN_State == 1))
        {
        }; // timeout is missing

        // Post Processing

        // RAW_Result = Read_Dword(RC_RAA_RD, 0x80);  // FDB_US_TOF_SUM_OF_ALL_U
        // RAW_Result = Read_Dword(RC_RAA_RD, 0x84);  // FDB_US_TOF_SUM_OF_ALL_D
        // RAW_Result /= DUT.Param.CR10.TOF_HIT_SUM_NO;  // Divided by number of hits

        RAW_Result = Read_Dword(RC_RAA_RD, 0x88); // FDB_US_TOF_0_U

        RAW_Result /= 65536; // divided by 2^16
        Time_Result = RAW_Result * 250 * (1e-9);

        Time_Result_ns = TIME_ns(Time_Result); // result in [ns]

        // Clear INTN
        Write_Opcode(RC_IF_CLR);

        delay(50); // used for debugging
    }
    /* USER CODE END 3 */
    return 0;
}