package main

import (
	"os"
	"syscall"
	"time"
)

const (
	// I2C_SLAVE ioctl request code to set the slave address
	I2C_SLAVE = 0x0703
	// Measure command as per your spec (AA_HEX)
	measureCmd = 0xAA
)

// Sensor represents the temperature/pressure sensor on an I²C bus.
type Sensor struct {
	file  *os.File
	delay time.Duration
}

// New opens "/dev/i2c-<bus>" (e.g. bus="1" → "/dev/i2c-1") and sets the 7-bit addr.
// measureDelay is the time the sensor needs to complete its conversion.
func New(bus string, addr uint8, measureDelay time.Duration) (*Sensor, error) {
	f, err := os.OpenFile("/dev/i2c-"+bus, os.O_RDWR, 0)
	if err != nil {
		return nil, err
	}
	// tell the kernel which slave we're talking to
	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL, f.Fd(), I2C_SLAVE, uintptr(addr)); errno != 0 {
		f.Close()
		return nil, errno
	}
	return &Sensor{file: f, delay: measureDelay}, nil
}

// readRaw issues the 0xAA “Measure” command, waits, then reads back
// 1 status byte + 3 bytes pressure + 3 bytes temperature.
func (s *Sensor) readRaw() (status byte, rawP, rawT uint32, err error) {
	// send measure command
	if _, err = s.file.Write([]byte{measureCmd}); err != nil {
		return
	}
	time.Sleep(s.delay)
	// read status + 3P + 3T
	buf := make([]byte, 1+3+3)
	if _, err = s.file.Read(buf); err != nil {
		return
	}
	status = buf[0]
	rawP = uint32(buf[1])<<16 | uint32(buf[2])<<8 | uint32(buf[3])
	rawT = uint32(buf[4])<<16 | uint32(buf[5])<<8 | uint32(buf[6])
	return
}

// GetPressure returns the latest pressure reading (in your desired units).
// Panics on I²C errors; swap to returning (float64, error) if you prefer.
func (s *Sensor) GetPressure() float64 {
	_, rawP, _, err := s.readRaw()
	if err != nil {
		panic(err)
	}
	return convertPressure(rawP)
}

// GetTemperature returns the latest temperature reading (°C).
// Panics on errors.
func (s *Sensor) GetTemperature() float64 {
	_, _, rawT, err := s.readRaw()
	if err != nil {
		panic(err)
	}
	return convertTemperature(rawT)
}

// Close frees the underlying file descriptor.
func (s *Sensor) Close() error {
	return s.file.Close()
}

// convertPressure applies your sensor’s raw→Pascals (or other) formula.
// Replace the stub below with the datasheet calibration.
func convertPressure(raw uint32) float64 {
	// e.g. return (float64(raw)/xyz - off) * scale
	return float64(raw)
}

// convertTemperature applies your raw→°C formula.
// Replace with the actual conversion from your datasheet.
func convertTemperature(raw uint32) float64 {
	// e.g. return float64(raw)/abc + offset
	return float64(raw)
}
