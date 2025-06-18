package main

import (
	"encoding/binary"
	"fmt"
	"log"
	"math"
	"os"
	"syscall"
	"unsafe"
)

var (
	spiFile    *os.File
	csHandleFd int
)

type gpiohandleRequest struct {
	LineOffsets   [64]uint32
	Flags         uint32
	DefaultValues [64]uint8
	ConsumerLabel [32]uint8
	Lines         uint32
	Fd            int32
}

type gpiohandleData struct {
	Values [64]uint8
}

type spiIocTransfer struct {
	TxBuf       uint64
	RxBuf       uint64
	Len         uint32
	SpeedHz     uint32
	DelayUsecs  uint16
	BitsPerWord uint8
	CsChange    uint8
	pad         uint32
}

// ioctl numbers from <linux/gpio.h>
const (
	GPIOHANDLE_REQUEST_OUTPUT        = 1 << 1
	GPIO_GET_LINEHANDLE_IOCTL        = 0xC16CB403 // _IOWR(0xB4, 0x03, struct gpiohandle_request)
	spiIOCWrMode                     = 0x40016b01 // _IOW('k', 1, __u8)
	spiIOCWrBitsPerWord              = 0x40016b03 // _IOW('k', 3, __u8)
	spiIOCWrMaxSpeedHz               = 0x40046b04 // _IOW('k', 4, __u32)
	GPIOHANDLE_SET_LINE_VALUES_IOCTL = 0xC040B409 // _IOW(0xB4, 0x09, struct gpiohandle_data)
	spiIOCMessage1                   = 0x40206b00 // _IOW('k', 0, struct spi_ioc_transfer)
)

// Global constants
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

// Adresses, opcodes and masks constants
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

// Global variables
var (
	myErrorCounter     uint32  = 0
	myNewConfiguration uint8   = 1
	myNewFHL           uint8   = 0
	myNewFHLmV         float32 = 0
	mySetFHLmV         float32 = 0
	myTOFSumAvgUP      float32
	myTOFSumAvgDOWN    float32
	myDiffTOFSumAvg    float32
	myTOFSumAvgUPNs    float32
	myTOFSumAvgDOWNNs  float32
	myDiffTOFSumAvgPs  float32
	myChipInitialized  uint8 = 0
	tofHitNO           uint32
	srrERRFLAGContent  uint32
)

// This data represents firmware configuration of AS6031 chip
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

// InitSPI initializes the SPI interface and configures the chip select (CS) GPIO line.
//
// It performs the following steps:
//   1. Opens the specified SPI device (e.g., "/dev/spidev0.0") for read/write access.
//   2. Configures the SPI mode, bits per word, and maximum speed using ioctl system calls.
//   3. Opens the specified GPIO chip character device (e.g., "/dev/gpiochip0").
//   4. Requests a handle for the specified CS GPIO line, configuring it as an output and setting its default value to high (idle).
//   5. Stores the file descriptors for the SPI device and CS GPIO handle for future use.
//
// Parameters:
//   - chipName: Name of the GPIO chip device (e.g., "gpiochip0").
//   - csGPIO: GPIO line number to use as the chip select (CS) signal.
//   - spiDev: Name of the SPI device (e.g., "spidev0.0").
//
// The function logs fatal errors and terminates the program if any step fails.
func InitSPI(chipName string, csGPIO int, spiDev string) {
	// 1) Open SPI device
	devPath := "/dev/" + spiDev
	f, err := os.OpenFile(devPath, os.O_RDWR, 0)
	if err != nil {
		log.Fatalf("Failed to open SPI device %s: %v", devPath, err)
	}
	spiFile = f

	// 2) Configure SPI mode/speed via ioctl
	var (
		mode1       = uint8(1)
		bits8       = uint8(8)
		speed500kHz = uint32(500000)
	)
	// mode
	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL, f.Fd(),
		spiIOCWrMode, uintptr(unsafe.Pointer(&mode1))); errno != 0 {
		log.Fatalf("Failed to set SPI mode: %v", errno)
	}
	// bits
	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL, f.Fd(),
		spiIOCWrBitsPerWord, uintptr(unsafe.Pointer(&bits8))); errno != 0 {
		log.Fatalf("Failed to set SPI bits-per-word: %v", errno)
	}
	// speed
	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL, f.Fd(),
		spiIOCWrMaxSpeedHz, uintptr(unsafe.Pointer(&speed500kHz))); errno != 0 {
		log.Fatalf("Failed to set SPI max speed: %v", errno)
	}

	// 3) Open the GPIO chip character device
	chipPath := "/dev/" + chipName
	chipDev, err := os.OpenFile(chipPath, os.O_RDWR, 0)
	if err != nil {
		log.Fatalf("Failed to open GPIO chip %s: %v", chipPath, err)
	}
	defer chipDev.Close()

	// 4) Prepare a line‐handle request for the CS line
	var req gpiohandleRequest
	req.LineOffsets[0] = uint32(csGPIO)   // which line on that chip
	req.Flags = GPIOHANDLE_REQUEST_OUTPUT // request as output
	req.DefaultValues[0] = 1              // idle-high
	copy(req.ConsumerLabel[:], "spi")     // label it “spi”
	req.Lines = 1

	// 5) Issue the ioctl to get the line handle fd
	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL, chipDev.Fd(),
		uintptr(GPIO_GET_LINEHANDLE_IOCTL), uintptr(unsafe.Pointer(&req))); errno != 0 {
		log.Fatalf("Failed to request GPIO line %d: %v", csGPIO, errno)
	}
	csHandleFd = int(req.Fd) // save for future toggles

	log.Printf("SPI initialized: device=%s, chip=%s, cs_gpio=%d\n",
		devPath, chipName, csGPIO)
}

func putCSLow() error {
	// prepare a single-line value-array with 0 (low)
	data := gpiohandleData{
		Values: [64]uint8{0},
	}

	// issue the ioctl on csHandleFd
	if _, _, errno := syscall.Syscall(
		syscall.SYS_IOCTL,
		uintptr(csHandleFd),
		uintptr(GPIOHANDLE_SET_LINE_VALUES_IOCTL),
		uintptr(unsafe.Pointer(&data)),
	); errno != 0 {
		return fmt.Errorf("failed to set CS low: %v", errno)
	}
	return nil
}

func putCSHigh() error {
	// prepare a single-line value-array with 1 (high)
	data := gpiohandleData{
		Values: [64]uint8{1},
	}

	// issue the ioctl on csHandleFd
	if _, _, errno := syscall.Syscall(
		syscall.SYS_IOCTL,
		uintptr(csHandleFd),
		uintptr(GPIOHANDLE_SET_LINE_VALUES_IOCTL),
		uintptr(unsafe.Pointer(&data)),
	); errno != 0 {
		return fmt.Errorf("failed to set CS high: %v", errno)
	}
	return nil
}

func spiTransfer(tx []byte, rx []byte) error {
	// determine how many bytes to clock
	length := len(tx)
	if len(rx) > length {
		length = len(rx)
	}

	// if we're reading-only, make a dummy TX buffer of the right size
	var dummy []byte
	if len(tx) == 0 && length > 0 {
		dummy = make([]byte, length)
		tx = dummy
	}

	var txPtr, rxPtr uintptr
	if len(tx) > 0 {
		txPtr = uintptr(unsafe.Pointer(&tx[0]))
	}
	if len(rx) > 0 {
		rxPtr = uintptr(unsafe.Pointer(&rx[0]))
	}

	msg := spiIocTransfer{
		TxBuf:       uint64(txPtr),
		RxBuf:       uint64(rxPtr),
		Len:         uint32(length),
		SpeedHz:     0, // leave as was configured
		DelayUsecs:  0, // leave as was configured
		BitsPerWord: 0, // leave as was configured
		CsChange:    0, // leave as was configured
	}

	if _, _, errno := syscall.Syscall(
		syscall.SYS_IOCTL,
		spiFile.Fd(),
		uintptr(spiIOCMessage1),
		uintptr(unsafe.Pointer(&msg)),
	); errno != 0 {
		return fmt.Errorf("ioctl SPI_IOC_MESSAGE failed: %v", errno)
	}
	return nil
}

// writeOpcode sends a single opcode byte over the SPI interface.
// It pulls the chip select (CS) line low before transmission and ensures
// the CS line is set high after the operation completes. If the SPI transfer
// fails, the function logs a fatal error and terminates the program.
//
// Parameters:
//   - b: The opcode byte to send over SPI.
func writeOpcode(b byte) {
	putCSLow()
	defer putCSHigh()

	tx := []byte{b}
	if err := spiTransfer(tx, nil); err != nil {
		log.Fatalf("SPI write failed: %v", err)
	}
}

// writeDword sends a 6-byte command over SPI consisting of an opcode, address, and a 32-bit value.
// The value is encoded in big-endian order.
// If the SPI transfer fails, the function logs a fatal error and terminates the program.
//
// Parameters:
//   - opcode:  The SPI command opcode byte.
//   - address: The target address byte.
//   - value:   The 32-bit value to write, encoded in big-endian order.
func writeDword(opcode, address byte, value uint32) {
	// build a 6-byte buffer: [opcode][address][value(4 bytes BE)]
	buf := make([]byte, 6)
	buf[0] = opcode
	buf[1] = address
	binary.BigEndian.PutUint32(buf[2:], value)

	putCSLow()
	defer putCSHigh()

	if err := spiTransfer(buf, nil); err != nil {
		log.Fatalf("SPI write failed: %v", err)
	}
}

// readDword sends an SPI command composed of the given opcode and address,
// then reads a 4-byte big-endian response from the SPI device and returns it as a uint32.
// If any SPI transfer fails, the function logs a fatal error and terminates the program.
//
// Parameters:
//   opcode  - the SPI command opcode to send
//   address - the address byte to send with the opcode
//
// Returns:
//   The 4-byte big-endian value read from the SPI device as a uint32.
func readDword(opcode, address byte) uint32 {
	tx := []byte{opcode, address}
	rx := make([]byte, 4)

	putCSLow()
	defer putCSHigh()

	// write the opcode+address
	if err := spiTransfer(tx, nil); err != nil {
		log.Fatalf("SPI write failed: %v", err)
	}
	// read the 4-byte response
	if err := spiTransfer(nil, rx); err != nil {
		log.Fatalf("SPI read failed: %v", err)
	}

	// decode big-endian uint32
	return binary.BigEndian.Uint32(rx)
}

// clearAllFlags resets all the relevant flags, namely Frontend Status, Error Flag, and Interrupt Flag.
func clearAllFlags() {
	writeDword(rcRAAWRRAM, byte(shrEXC), (fesCLRMask | efCLRMask | ifCLRMask))
}

// twoSComplementConversion converts a raw unsigned 16-bit sensor value (provided as uint32)
// into a signed floating-point value using two's complement representation and applies a
// multiplication factor.
//
// Parameters:
//   - rawNumber: The raw sensor value as an unsigned 32-bit integer.
//   - multFactor: A floating-point multiplier to scale the converted value.
//
// Returns:
//   - The converted and scaled value as a float32.
func twoSComplementConversion(rawNumber uint32, multFactor float32) float32 {
	var exp float64
	var halfExp float64
	var number float64
	exp = 65536

	halfExp = exp / 2
	number = float64(rawNumber) / exp

	if number > halfExp-1 {
		number -= exp
	}

	return float32(number * float64(multFactor))
}

// writeSensorConfig writes a sequence of 32-bit configuration values to a sensor over SPI,
// starting from a specified address and using an auto-incrementing address mode.
//
// Parameters:
//   opcode   - The SPI command opcode to initiate the write operation.
//   fromAddr - The starting register address to write to.
//   data     - A slice of 32-bit unsigned integers to write to the sensor.
//   toAddr   - The ending register address (inclusive).
//
// It sends an auto-increment header followed by the data words in
// big-endian order. If the data slice is too small for the address range, or if any SPI
// transfer fails, the function logs a fatal error and terminates the program.
func writeSensorConfig(opcode uint8, fromAddr int, data []uint32, toAddr int) {
	putCSLow()
	defer putCSHigh()

	// Write the auto-increment header: [opcode][start address]
	header := []byte{opcode, byte(fromAddr)}
	if err := spiTransfer(header, nil); err != nil {
		log.Fatalf("Failed to write auto-incr header: %v", err)
	}

	// Ensure data slice is large enough for the address range
	// (inclusive count = toAddr - fromAddr + 1)
	count := toAddr - fromAddr + 1
	if count > len(data) {
		log.Fatalf("Data array is smaller than the address range")
	}

	// Write each 32-bit word in big-endian order
	for i := 0; i < count; i++ {
		buf := make([]byte, 4)
		binary.BigEndian.PutUint32(buf, data[i])
		if err := spiTransfer(buf, nil); err != nil {
			log.Fatalf("Failed to write data at address %d: %v", fromAddr+i, err)
		}
	}
}

// SensorInit initializes the sensor by configuring its registers with predefined values.
// It clears all status flags, sets up the sensor's configuration registers, and writes
// specific control words to the sensor. This function prepares the sensor for operation
// by enabling required features, setting timing parameters, and applying necessary trims.
// The configuration is written via a sequence of opcodes and register writes.
func SensorInit() {
	cfgRegisters := [20]uint32{
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
		0x0000004B, // [0xDB] SHR_ZCD_FHL_D
	}

	clearAllFlags()

	tofHitNO = cfgRegisters[10] & uint32(tofHitNOMask)
	tofHitNO >>= 8

	writeOpcode(rcBMREQ)
	writeOpcode(rcMCTOFF)
	writeSensorConfig(rcRAAWRRAM, 0xC0, cfgRegisters[:], 0xCF)
	writeDword(rcRAAWRRAM, byte(shrTOFRate), 0x00000001)
	writeDword(rcRAAWRRAM, byte(shrUSMRLSDLYU), 0x000015E0)
	writeDword(rcRAAWRRAM, byte(shrUSMRLSDLYD), 0x000015E0)
	writeDword(rcRAAWRRAM, byte(shrZCDFHLU), 0x0000004B)
	writeDword(rcRAAWRRAM, byte(shrZCDFHLD), 0x0000004B)
	writeOpcode(rcMCTON)
	writeOpcode(rcIFCLR)
	writeOpcode(rcBMRLS)
}

// myInitState initializes the state variables by resetting the error counter,
// clearing the new configuration flag, and marking the chip as initialized.
func myInitState() {
	myErrorCounter = 0
	myNewConfiguration = 0
	myChipInitialized = 1
}

// calcTimeOfFlight calculates the time of flight for a sensor at the specified address.
// It reads a 32-bit raw value from the sensor's memory, converts it using two's complement conversion,
// and returns the result as a float32 representing the time of flight.
//
// Parameters:
//   tofAddress - The address of the time-of-flight sensor.
//
// Returns:
//   float32 - The calculated time of flight.
func calcTimeOfFlight(tofAddress byte) float32 {
	var rawValue uint32
	var floatValue float32

	rawValue = readDword(rcRAARDRAM, tofAddress)
	floatValue = twoSComplementConversion(rawValue, float32(tRef))

	return floatValue
}

// ReadFlowRate reads sensor data to calculate and return the volumetric flow rate.
// It checks for error flags, processes time-of-flight measurements, and computes the flow rate
// using physical constants and sensor parameters. If an error is detected, it clears flags and returns -1.0.
// Returns the calculated volumetric flow rate as a float64 value.
func ReadFlowRate() float64 {

	srrERRFLAGContent = readDword(rcRAARDRAM, srrERRFLAG)

	if srrERRFLAGContent > 0 {
		//fmt.Printf("SRR_ERR_FLAG_content%d\n", srrERRFLAGContent)
		fmt.Println("...error!")
		myErrorCounter++
		clearAllFlags()
	} else {
		myTOFSumAvgDOWN = calcTimeOfFlight(byte(fdbUSTOFADDALLD)) / float32(tofHitNO)
		myTOFSumAvgUP = calcTimeOfFlight(byte(fdbUSTOFADDALLU)) / float32(tofHitNO)

		myDiffTOFSumAvg = myTOFSumAvgDOWN - myTOFSumAvgUP

		myTOFSumAvgUPNs = myTOFSumAvgUP / 1e-9
		myTOFSumAvgDOWNNs = myTOFSumAvgDOWN / 1e-9
		myDiffTOFSumAvgPs = myDiffTOFSumAvg / 1e-12

		velocity := (math.Abs(float64(myDiffTOFSumAvgPs)) * (speedOfSoundWater * speedOfSoundWater)) / (2 * lenOfSens)
		velocity *= 1e-12
		volumetricFlowRate := vfrConstant * kFact * velocity * crossArea

		clearAllFlags()
		return volumetricFlowRate
	}
	clearAllFlags()
	return -1.0
}
