package main

import (
	"flag"
	"fmt"
	"math"
	"os"
	"time"

	"main/spi_interface"
)

// Constants (same as before)
const (
	defaultMeasDelaySec = 1
	defaultCSGPIO       = 25
	defaultSPIPath      = "/dev/spidev0.0"
	defaultSPIChipName  = "gpiochip0"
	speedOfSoundWater   = 1480.0
	lenOfSens           = 0.06456
	crossArea           = 0.000501653
	kFact               = 1.008
	vfrConstant         = 15850.32
	hsCkolck            = 4e6
	tRef                = 1.0 / hsCkolck
)

// Masks (same as before)
const (
	tofHitNOMask    uint32 = 0x00001F00
	fesCLRMask      uint32 = 1 << 2
	efCLRMask       uint32 = 1 << 1
	ifCLRMask       uint32 = 1 << 0
	usAMUPDMask     uint32 = 1 << 8
	rcBMREQ         byte   = 0x88
	rcMCTOFF        byte   = 0x8A
	rcRAAWRRAM      byte   = 0x5A
	rcRAARDRAM      byte   = 0x7A
	shrTOFRate      byte   = 0xD0
	shrUSMRLSDLYU   byte   = 0xD1
	shrUSMRLSDLYD   byte   = 0xD3
	shrZCDFHLU      byte   = 0xDA
	shrZCDFHLD      byte   = 0xDB
	shrEXC          byte   = 0xDD
	rcMCTON         byte   = 0x8B
	rcIFCLR         byte   = 0x8D
	rcBMRLS         byte   = 0x87
	fdbUSAMCVH      byte   = 0x83
	fdbUSAMCVL      byte   = 0x87
	fdbUSAMU        byte   = 0x82
	fdbUSAMD        byte   = 0x86
	fdbUSTOFADDALLU byte   = 0x80
	fdbUSTOFADDALLD byte   = 0x84
	fdbUSPWU        byte   = 0x81
	fdbUSPWD        byte   = 0x85
	srrERRFLAG      byte   = 0xE1
	shrFHLU         byte   = 0xDA
	shrFHLD         byte   = 0xDB
)

// Global variables (same as before)
var (
	velocity           float64
	volumetricFlowRate float64
	myErrorCounter     uint32  = 0
	myNewConfiguration uint8   = 1
	myNewFHL           uint8   = 0
	myNewFHLmV         float32 = 0
	mySetFHLmV         float32 = 0
	myRAWValueUP       uint32
	myRAWValueDOWN     uint32
	myRAWAMCVH         uint32
	myRAWAMCVL         uint32
	myRAWPWUP          uint32
	myRAWPWDOWN        uint32
	myTOFSumAvgUP      float32
	myTOFSumAvgDOWN    float32
	myDiffTOFSumAvg    float32
	myRealAMUP         float32
	myRealAMDOWN       float32
	myRealPWUP         float32
	myRealPWDOWN       float32
	myTOFSumAvgUPNs    float32
	myTOFSumAvgDOWNNs  float32
	myDiffTOFSumAvgPs  float32
	myChipInitialized  uint8 = 0
	myChipIdleState    uint8 = 0
	tofHitNO           uint32
	srrERRFLAGContent  uint32
	srrFEPSTFContent   uint32 // Assuming this is read somewhere
)

var fwc = []byte{
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
	0xCD,
}

func twoSComplementConversion(rawNumber uint32, bit int, multFactor float32) float32 {
	var exp float64
	var halfExp float64
	var number float64

	if bit == 32 {
		exp = 4294967296
	} else if bit == 24 {
		exp = 16777216
	} else if bit == 16 {
		exp = 65536
	} else if bit == 8 {
		exp = 256
	}

	halfExp = exp / 2
	number = float64(rawNumber) / exp

	if number > halfExp-1 {
		number -= exp
	}

	return float32(number * float64(multFactor))
}

func writeConfig() {
	cfgRegisters := [20]uint32{
		0x48DBA399,
		0x00800401,
		0x00000000,
		0x00000001,
		0x0011F7FF,
		0x6046EF29,
		0x01012100,
		0x00240000,
		0x006807E4,
		0x60160204,
		0x010FEA14,
		0x23A4DE81,
		0x94A0C46C,
		0x401100C4,
		0x00A7400F,
		0x00000001,
		0x000015E0,
		0x000015E0,
		0x0000004B,
		0x0000004B,
	}
	tofHitNO = cfgRegisters[10] & uint32(tofHitNOMask)
	tofHitNO >>= 8

	fmt.Println("Writing configuration...")
	spi_interface.WriteOpcode(rcBMREQ)
	spi_interface.WriteOpcode(rcMCTOFF)
	spi_interface.WriteRegisterAutoIncr(rcRAAWRRAM, 0xC0, cfgRegisters[:], 0xCF)
	spi_interface.WriteDword(rcRAAWRRAM, byte(shrTOFRate), 0x00000001)
	spi_interface.WriteDword(rcRAAWRRAM, byte(shrUSMRLSDLYU), 0x000015E0)
	spi_interface.WriteDword(rcRAAWRRAM, byte(shrUSMRLSDLYD), 0x000015E0)
	spi_interface.WriteDword(rcRAAWRRAM, byte(shrZCDFHLU), 0x0000004B)
	spi_interface.WriteDword(rcRAAWRRAM, byte(shrZCDFHLD), 0x0000004B)
	spi_interface.WriteOpcode(rcMCTON)
	spi_interface.WriteOpcode(rcIFCLR)
	spi_interface.WriteOpcode(rcBMRLS)
}

func calcTimeOfFlight(tofAddress byte) float32 {
	var rawValue uint32
	var floatValue float32

	fmt.Printf("Reading TOF from address: 0x%X\n", tofAddress)
	rawValue = spi_interface.ReadDword(rcRAARDRAM, tofAddress)
	floatValue = twoSComplementConversion(rawValue, 16, float32(tRef))

	return floatValue
}

func myInitState() {
	myErrorCounter = 0
	myNewConfiguration = 0
	myChipInitialized = 1
}

func calcAmplitude(amAddress byte, amcVH uint32, amcVL uint32) float32 {
	var rawValue uint32
	var amcGradient float32
	var amcOffset float32
	var floatValue float32

	fmt.Printf("Reading Amplitude from address: 0x%X\n", amAddress)
	rawValue = spi_interface.ReadDword(rcRAARDRAM, amAddress)
	amcGradient = 350.0 / float32(amcVH-amcVL)
	amcOffset = (float32(2*amcVL) - float32(amcVH)) * amcGradient
	floatValue = (amcGradient * float32(rawValue)) - amcOffset

	return floatValue
}

func processTOF() {
	fmt.Println("Process_TOF")

	srrERRFLAGContent = spi_interface.ReadDword(rcRAARDRAM, srrERRFLAG)

	if srrERRFLAGContent > 0 {
		fmt.Printf("SRR_ERR_FLAG_content%d\n", srrERRFLAGContent)
		fmt.Println("...error!")
		myErrorCounter++
		spi_interface.WriteDword(rcRAAWRRAM, byte(shrEXC), (fesCLRMask | efCLRMask))
		srrERRFLAGContent = spi_interface.ReadDword(rcRAARDRAM, srrERRFLAG)
	} else {
		if (srrFEPSTFContent & usAMUPDMask) != 0 {
			if myRAWAMCVH == 0 || myRAWAMCVL == 0 {
				myRAWAMCVH = spi_interface.ReadDword(rcRAARDRAM, byte(fdbUSAMCVH))
				myRAWAMCVL = spi_interface.ReadDword(rcRAARDRAM, byte(fdbUSAMCVL))
				fmt.Printf("MyRAWAMCVH%d MyRAWAMCVL%d\n", myRAWAMCVH, myRAWAMCVL)
			}

			if myRAWAMCVH != 0 && myRAWAMCVL != 0 {
				myRealAMUP = calcAmplitude(byte(fdbUSAMU), myRAWAMCVH, myRAWAMCVL)
				myRealAMDOWN = calcAmplitude(byte(fdbUSAMD), myRAWAMCVH, myRAWAMCVL)
				fmt.Printf("MyRealAMUP%f MyRealAMDOWN%f\n", myRealAMUP, myRealAMDOWN)
			}
		}

		myTOFSumAvgUP = calcTimeOfFlight(byte(fdbUSTOFADDALLU)) / float32(tofHitNO)
		myTOFSumAvgDOWN = calcTimeOfFlight(byte(fdbUSTOFADDALLD)) / float32(tofHitNO)

		myRAWPWUP = spi_interface.ReadDword(rcRAARDRAM, byte(fdbUSPWU))
		myRAWPWDOWN = spi_interface.ReadDword(rcRAARDRAM, byte(fdbUSPWD))

		myDiffTOFSumAvg = myTOFSumAvgDOWN - myTOFSumAvgUP

		myRealPWUP = float32(myRAWPWUP) / (1 << 7)
		myRealPWDOWN = float32(myRAWPWDOWN) / (1 << 7)

		myTOFSumAvgUPNs = myTOFSumAvgUP / 1e-9
		myTOFSumAvgDOWNNs = myTOFSumAvgDOWN / 1e-9
		myDiffTOFSumAvgPs = myDiffTOFSumAvg / 1e-12

		velocity = (math.Abs(float64(myDiffTOFSumAvgPs)) * (speedOfSoundWater * speedOfSoundWater)) / (2 * lenOfSens)
		velocity *= 1e-12
		volumetricFlowRate = vfrConstant * kFact * velocity * crossArea

		fmt.Printf("TOF data: %.3f\t%.3f\t%.3f\n", myTOFSumAvgUPNs, myTOFSumAvgDOWNNs, myDiffTOFSumAvgPs)
		fmt.Printf("Velocity: %f\n", velocity)
		fmt.Printf("Volumetric Flow Rate: %f\n", volumetricFlowRate)

	}
	spi_interface.WriteDword(rcRAAWRRAM, byte(shrEXC), (fesCLRMask | efCLRMask | ifCLRMask))
}

func main() {
	fmt.Println("main!")

	var csGpio uint
	var measDelay float64
	var spiDevicePath string
	var chipName string

	flag.UintVar(&csGpio, "c", defaultCSGPIO, "CS GPIO pin")
	flag.Float64Var(&measDelay, "d", defaultMeasDelaySec, "Measurement delay in seconds")
	flag.StringVar(&spiDevicePath, "p", defaultSPIPath, "SPI device path")
	flag.StringVar(&chipName, "n", defaultSPIChipName, "GPIO chip name")
	flag.Parse()

	if csGpio < 0 {
		fmt.Fprintf(os.Stderr, "Invalid CS GPIO. Using default: %d\n", defaultCSGPIO)
		csGpio = defaultCSGPIO
	}
	if measDelay <= 0 {
		fmt.Fprintf(os.Stderr, "Invalid measurement delay. Using default: %.2f seconds\n", float64(defaultMeasDelaySec))
		measDelay = defaultMeasDelaySec
	}

	fmt.Printf("Starting with CS GPIO: %d, Measurement delay: %.2f seconds, SPI device: %s, GPIO chip: %s\n", csGpio, measDelay, spiDevicePath, chipName)

	spi_interface.InitSPI(chipName, int(csGpio), spiDevicePath)

	spi_interface.WriteDword(rcRAAWRRAM, byte(shrEXC), (fesCLRMask | efCLRMask | ifCLRMask))

	for {
		fmt.Println("while main!")
		time.Sleep(time.Duration(measDelay) * time.Second)

		if myChipInitialized == 1 {
			fmt.Println("(My_Chip_initialized == 1)")
			processTOF()

			/* Update Configuration */
			if myNewFHLmV > 0 {
				fmt.Println("My_New_FHL_mV > 0")
				/* Update System Handling Register
				 * SHR_FHL_U (First Hit Level Up) 0x0DA
				 * SHR_FHL_D (First Hit Level Down) 0x0DB
				 */
				myNewFHL = uint8(myNewFHLmV / 0.88)

				spi_interface.WriteDword(rcRAAWRRAM, byte(shrFHLU), uint32(myNewFHL))
				spi_interface.WriteDword(rcRAAWRRAM, byte(shrFHLD), uint32(myNewFHL))

				mySetFHLmV = myNewFHLmV
				myNewFHLmV = 0
			}
		}

		/* Reload Configuration */
		if myNewConfiguration > 0 {
			fmt.Println("My_New_Configuration > 0")
			myChipInitialized = 0

			// Put_UFC_Into_Idle();
			myChipIdleState = 1

			if myNewConfiguration != 99 {
				fmt.Println("My_New_Configuration != 99")
				writeConfig() // Write Configuration to AS6031

				mySetFHLmV = float32(spi_interface.ReadDword(rcRAARDRAM, byte(shrZCDFHLU))) * 0.88

				spi_interface.WriteOpcode(rcMCTON) // RC_MCT_ON
				// Write_Opcode(RC_IF_CLR);
				myChipIdleState = 0
				myInitState()
			}
		}
	} // End of while loop
}
