package API

import (
	"encoding/binary"
	"flag"
	"fmt"
	"github.com/warthog618/go-gpiocdev"
	"log"
	"math"
	"os"
	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
	"periph.io/x/conn/v3/spi/spireg"
	"periph.io/x/host/v3"
	"time"
)

// spiPort is the SPI connection to the sensor.
var (
	spiPort spi.Conn
	csLine  *gpiocdev.Line
)

// Constants (same as before)
const (
	defaultMeasDelaySec = 1
	defaultCSGPIO       = 25
	defaultSPIPath      = "spidev0.0"
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
	shrUSMRLSDLYD   byte   = 0xD2
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

func putCSLow() {
	if err := csLine.SetValue(0); err != nil {
		log.Fatalf("Failed to set CS low: %v", err)
	}
}

func putCSHigh() {
	if err := csLine.SetValue(1); err != nil {
		log.Fatalf("Failed to set CS high: %v", err)
	}
}

func WriteOpcode(b byte) {
	putCSLow()
	defer putCSHigh()
	if err := spiPort.Tx([]byte{b}, nil); err != nil {
		log.Fatalf("SPI write failed: %v", err)
	}
}

func WriteDword(opcode, address byte, value uint32) {
	buf := []byte{opcode, address}
	tmp := make([]byte, 4)
	binary.BigEndian.PutUint32(tmp, value)
	buf = append(buf, tmp...)

	putCSLow()
	defer putCSHigh()
	if err := spiPort.Tx(buf, nil); err != nil {
		log.Fatalf("SPI write failed: %v", err)
	}
}

func ReadDword(opcode, address byte) uint32 {
	tx := []byte{opcode, address}
	rx := make([]byte, 4)

	putCSLow()
	defer putCSHigh()

	if err := spiPort.Tx(tx, nil); err != nil {
		log.Fatalf("SPI write failed: %v", err)
	}
	if err := spiPort.Tx(nil, rx); err != nil {
		log.Fatalf("SPI read failed: %v", err)
	}

	return binary.BigEndian.Uint32(rx)
}

// InitSPI initializes the SPI interface and CS GPIO line.
func InitSPI(chipName string, csGPIO int, spiDev string) {
	if _, err := host.Init(); err != nil {
		log.Fatalf("Failed to initialize periph: %v", err)
	}

	// Open SPI device
	spiDevPath := fmt.Sprintf("/dev/%s", spiDev)
	port, err := spireg.Open(spiDevPath)
	if err != nil {
		log.Fatalf("Failed to open SPI device: %v", err)
	}

	// Configure SPI port
	spiSpeed := 500 * physic.KiloHertz
	conn, err := port.Connect(spiSpeed, spi.Mode1, 8)
	if err != nil {
		log.Fatalf("Failed to configure SPI port: %v", err)
	}
	spiPort = conn

	// Open GPIO chip
	chip, err := gpiocdev.NewChip(chipName)
	if err != nil {
		log.Fatalf("Failed to open GPIO chip %s: %v", chipName, err)
	}
	defer chip.Close()

	// Request CS line
	line, err := chip.RequestLine(csGPIO,
		gpiocdev.AsOutput(1), // ✅ correct usage
		gpiocdev.WithConsumer("spi"),
	)
	if err != nil {
		log.Fatalf("Failed to request CS line %d: %v", csGPIO, err)
	}
	csLine = line

	log.Printf("SPI initialized: device=%s, chip=%s, cs_gpio=%d\n", spiDevPath, chipName, csGPIO)
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


func writeSensorConfig(opcode uint8, fromAddr int, data []uint32, toAddr int) {
	putCSLow()
	defer putCSHigh()

	if err := spiPort.Tx([]byte{opcode, byte(fromAddr)}, nil); err != nil {
		log.Fatalf("Failed to write auto-incr header: %v", err)
	}

	for addr := int(fromAddr); addr <= toAddr; addr++ {
		if (toAddr-fromAddr) >= len(data) {
			log.Fatalf("Data array is smaller than the address range")
		}

		buf := make([]byte, 4)
		binary.BigEndian.PutUint32(buf, data[addr-fromAddr])
		if err := spiPort.Tx(buf, nil); err != nil {
			log.Fatalf("Failed to write dword in auto-incr: %v", err)
		}
	}
}

func sensorInit() {
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

	myNewFHL = uint8(myNewFHLmV / 0.88)

	WriteDword(rcRAAWRRAM, byte(shrFHLU), uint32(myNewFHL))
	WriteDword(rcRAAWRRAM, byte(shrFHLD), uint32(myNewFHL))

	mySetFHLmV = myNewFHLmV
	myNewFHLmV = 0

	tofHitNO = cfgRegisters[10] & uint32(tofHitNOMask)
	tofHitNO >>= 8

	tofHitNO = cfgRegisters[10] & uint32(tofHitNOMask)
	tofHitNO >>= 8

	fmt.Println("Writing configuration...")
	WriteOpcode(rcBMREQ)
	WriteOpcode(rcMCTOFF)
	fmt.Println("1111")
	writeSensorConfig(rcRAAWRRAM, 0xC0, cfgRegisters[:], 0xCF)
	WriteDword(rcRAAWRRAM, byte(shrTOFRate), 0x00000001)
	WriteDword(rcRAAWRRAM, byte(shrUSMRLSDLYU), 0x000015E0)
	WriteDword(rcRAAWRRAM, byte(shrUSMRLSDLYD), 0x000015E0)
	WriteDword(rcRAAWRRAM, byte(shrZCDFHLU), 0x0000004B)
	WriteDword(rcRAAWRRAM, byte(shrZCDFHLD), 0x0000004B)
	WriteOpcode(rcMCTON)
	WriteOpcode(rcIFCLR)
	WriteOpcode(rcBMRLS)
}

func calcTimeOfFlight(tofAddress byte) float32 {
	var rawValue uint32
	var floatValue float32

	fmt.Printf("Reading TOF from address: 0x%X\n", tofAddress)
	rawValue = ReadDword(rcRAARDRAM, tofAddress)
	floatValue = twoSComplementConversion(rawValue, 16, float32(tRef))
	fmt.Println("TOF raw value:", rawValue)
	fmt.Println("TOF float value:", floatValue)

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
	rawValue = ReadDword(rcRAARDRAM, amAddress)
	amcGradient = 350.0 / float32(amcVH-amcVL)
	amcOffset = (float32(2*amcVL) - float32(amcVH)) * amcGradient
	floatValue = (amcGradient * float32(rawValue)) - amcOffset

	return floatValue
}

func readTOF() {
	fmt.Println("readTOF")

	srrERRFLAGContent = ReadDword(rcRAARDRAM, srrERRFLAG)

	if srrERRFLAGContent > 0 {
		fmt.Printf("SRR_ERR_FLAG_content%d\n", srrERRFLAGContent)
		fmt.Println("...error!")
		myErrorCounter++
		WriteDword(rcRAAWRRAM, byte(shrEXC), (fesCLRMask | efCLRMask))
		srrERRFLAGContent = ReadDword(rcRAARDRAM, srrERRFLAG)
	} else {
		myTOFSumAvgDOWN = calcTimeOfFlight(byte(fdbUSTOFADDALLD)) / float32(tofHitNO)
		myTOFSumAvgUP = calcTimeOfFlight(byte(fdbUSTOFADDALLU)) / float32(tofHitNO)

		myDiffTOFSumAvg = myTOFSumAvgDOWN - myTOFSumAvgUP

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
	WriteDword(rcRAAWRRAM, byte(shrEXC), (fesCLRMask | efCLRMask | ifCLRMask))
}
