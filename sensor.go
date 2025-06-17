// sensor.go – minimal-dep rewrite, Go ≥1.20
package main

import (
	"encoding/binary"
	"fmt"
	"log"
	"math"
	"os"
	"strconv"
	"syscall"   // std-lib only!
	"time"
	"unsafe"
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

// hand-coded spi/spidev.h numbers for ARM64 Linux
const (
	SPI_IOC_WR_MODE          = 0x40016b01
	SPI_IOC_WR_BITS_PER_WORD = 0x40016b03
	SPI_IOC_WR_MAX_SPEED_HZ  = 0x40046b04
)

const SPI_IOC_MESSAGE_1 = 0x40206b00
type spiIOCTransfer struct {
	txBuf        uint64
	rxBuf        uint64
	length       uint32
	speedHz      uint32
	delayUsecs   uint16
	bitsPerWord  uint8
	csChange     uint8
	txNBits      uint8
	rxNBits      uint8
	pad          uint16
}

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

/* --------------------------- small helpers --------------------------- */

func spiTransfer(fd int, tx, rx []byte) error {
	var t spiIOCTransfer
	if len(tx) > 0 {
		t.txBuf = uint64(uintptr(unsafe.Pointer(&tx[0])))
	}
	if len(rx) > 0 {
		t.rxBuf = uint64(uintptr(unsafe.Pointer(&rx[0])))
	}
	if n := len(tx); n > 0 {
		t.length = uint32(n)
	} else {
		t.length = uint32(len(rx))
	}
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL,
		uintptr(fd),
		uintptr(SPI_IOC_MESSAGE_1),
		uintptr(unsafe.Pointer(&t)),
	)
	if errno != 0 {
		return errno
	}
	return nil
}

func ioctlSetInt(fd uintptr, req, value int) error {
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, fd, uintptr(req), uintptr(value))
	if errno != 0 {
		return errno
	}
	return nil
}

// simple sysfs-GPIO bits – enough for manual CS
func gpioExport(pin int) error {
	return os.WriteFile("/sys/class/gpio/export",
		[]byte(strconv.Itoa(pin)), 0o644)
}
func gpioDirection(pin int, out bool) error {
	dir := "in"
	if out {
		dir = "out"
	}
	return os.WriteFile(
		fmt.Sprintf("/sys/class/gpio/gpio%d/direction", pin),
		[]byte(dir), 0o644)
}
func gpioWrite(pin int, hi bool) error {
	val := "0"
	if hi {
		val = "1"
	}
	return os.WriteFile(
		fmt.Sprintf("/sys/class/gpio/gpio%d/value", pin),
		[]byte(val), 0o644)
}

/* --------------------------- SPI wrapper ----------------------------- */

type spiDev struct{ fd int }

func openSPI(path string, mode uint8, speedHz uint32) (*spiDev, error) {
	fd, err := syscall.Open(path, syscall.O_RDWR, 0)
	if err != nil {
		return nil, err
	}
	if err := ioctlSetInt(uintptr(fd), SPI_IOC_WR_MODE, int(mode)); err != nil {
		return nil, err
	}
	if err := ioctlSetInt(uintptr(fd), SPI_IOC_WR_BITS_PER_WORD, 8); err != nil {
		return nil, err
	}
	if err := ioctlSetInt(uintptr(fd), SPI_IOC_WR_MAX_SPEED_HZ, int(speedHz)); err != nil {
		return nil, err
	}
	return &spiDev{fd: fd}, nil
}

func (s *spiDev) Tx(tx, rx []byte) error { return spiTransfer(s.fd, tx, rx) }
func (s *spiDev) Close()                 { _ = syscall.Close(s.fd) }


/* --------------------------- driver state ---------------------------- */

var (
	spiPort *spiDev
	csPin   = 25 // default
)

/* --------------------------- public init ----------------------------- */

func writeSensorConfig(op byte, start int, data []uint32, end int) {
	csLow()
	defer csHigh()
	// write header
	_ = spiPort.Tx([]byte{op, byte(start)}, nil)
	// write words
	for addr := start; addr <= end && addr-start < len(data); addr++ {
		var tmp [4]byte
		binary.BigEndian.PutUint32(tmp[:], data[addr-start])
		_ = spiPort.Tx(tmp[:], nil)
	}
}

// clearAllFlags = reset FES / EF / IF bits in EXC
func clearAllFlags() {
	writeDword(rcRAAWRRAM, shrEXC, fesCLRMask|efCLRMask|ifCLRMask)
}

// calcTimeOfFlight returns a signed float value from a RAM address
func calcTimeOfFlight(addr byte) float32 {
	raw := readDword(rcRAARDRAM, addr)
	return twos(raw, tRef)
}

func InitSensor(spiPath string, cs int) {
	csPin = cs

	/* ---- GPIO (CS) ---- */
	_ = gpioExport(csPin) // ignore “already exported”
	time.Sleep(10 * time.Millisecond)
	must(gpioDirection(csPin, true))
	must(gpioWrite(csPin, true)) // idle high

	/* ---- SPI ---- */
	const mode1 uint8 = 0x01 // CPOL=0, CPHA=1
	dev, err := openSPI("/dev/"+spiPath, mode1, 500_000)
	must(err)
	spiPort = dev
}

/* --------------------------- hw helpers ------------------------------ */

func csLow()  { must(gpioWrite(csPin, false)) }
func csHigh() { must(gpioWrite(csPin, true)) }

func writeOpcode(b byte) {
	csLow()
	defer csHigh()
	must(spiPort.Tx([]byte{b}, nil))
}

func writeDword(op, addr byte, v uint32) {
	buf := []byte{op, addr, 0, 0, 0, 0}
	binary.BigEndian.PutUint32(buf[2:], v)
	csLow()
	defer csHigh()
	must(spiPort.Tx(buf, nil))
}

func readDword(op, addr byte) uint32 {
	tx := []byte{op, addr}
	rx := make([]byte, 4)
	csLow()
	defer csHigh()
	must(spiPort.Tx(tx, nil))
	must(spiPort.Tx(nil, rx))
	return binary.BigEndian.Uint32(rx)
}

func twos(raw uint32, mult float32) float32 {
	const full = 65536.0
	val := float64(raw)
	if val > full/2-1 {
		val -= full
	}
	return float32(val/full) * mult
}

/* --------------------------- high-level API -------------------------- */

func SensorInit() {
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

	writeDword(rcRAAWRRAM, byte(shrFHLU), uint32(myNewFHL))
	writeDword(rcRAAWRRAM, byte(shrFHLD), uint32(myNewFHL))

	mySetFHLmV = myNewFHLmV
	myNewFHLmV = 0

	tofHitNO = cfgRegisters[10] & uint32(tofHitNOMask)
	tofHitNO >>= 8

	tofHitNO = cfgRegisters[10] & uint32(tofHitNOMask)
	tofHitNO >>= 8

	//fmt.Println("Writing configuration...")
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

		//fmt.Printf("TOF data: %.3f\t%.3f\t%.3f\n", myTOFSumAvgUPNs, myTOFSumAvgDOWNNs, myDiffTOFSumAvgPs)
		//fmt.Printf("Velocity: %f\n", velocity)
		//fmt.Printf("Volumetric Flow Rate: %f\n", volumetricFlowRate)
		clearAllFlags()
		return volumetricFlowRate
	}
	clearAllFlags()
	return 0.0
}

/* --------------------------- utils ---------------------------------- */

func must(err error) {
	if err != nil {
		log.Fatal(err)
	}
}
