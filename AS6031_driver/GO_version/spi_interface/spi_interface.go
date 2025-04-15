package spi_interface

import (
	"encoding/binary"
	"fmt"
	"log"

	"github.com/warthog618/go-gpiocdev"
	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
	"periph.io/x/conn/v3/spi/spireg"
	"periph.io/x/host/v3"
)

var (
	spiPort spi.Conn
	csLine  *gpiocdev.Line
)

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

func WriteRegisterAutoIncr(opcode, fromAddr byte, data []uint32, toAddr int) {
	if len(data) != int(toAddr)-int(fromAddr)+1 {
		log.Fatalf("Mismatch between data length and address range")
	}

	putCSLow()
	defer putCSHigh()

	if err := spiPort.Tx([]byte{opcode, fromAddr}, nil); err != nil {
		log.Fatalf("Failed to write auto-incr header: %v", err)
	}

	for _, val := range data {
		buf := make([]byte, 4)
		binary.BigEndian.PutUint32(buf, val)
		if err := spiPort.Tx(buf, nil); err != nil {
			log.Fatalf("Failed to write dword in auto-incr: %v", err)
		}
	}
}
