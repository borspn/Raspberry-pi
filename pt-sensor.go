package main

import (
	"fmt"
	"os"
	"syscall"
	"time"
)

const (
	I2C_SLAVE  = 0x0703
	measureCmd = 0xAA
)

type Sensor struct {
	file  *os.File
	delay time.Duration
}

func New(bus string, addr uint8, measureDelay time.Duration) (*Sensor, error) {
	f, err := os.OpenFile("/dev/i2c-"+bus, os.O_RDWR, 0)
	if err != nil {
		return nil, err
	}
	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL, f.Fd(), I2C_SLAVE, uintptr(addr)); errno != 0 {
		f.Close()
		return nil, errno
	}
	return &Sensor{file: f, delay: measureDelay}, nil
}

type tempOutput struct {
	bite1 byte
	bite2 byte
	bite3 byte
}

func (s *Sensor) readRaw() (status byte, rawP, rawT uint32, err error) {
	if _, err = s.file.Write([]byte{measureCmd}); err != nil {
		return
	}
	time.Sleep(s.delay)
	buf := make([]byte, 1+3+3)
	if _, err = s.file.Read(buf); err != nil {
		return
	}
	status = buf[0]
	fmt.Println("Status:", status)
	for i := 0; i < 6; i++ {
		fmt.Printf("Byte %d: %x\n", i+1, buf[i+1])
	}
	rawP = uint32(buf[1])<<16 | uint32(buf[2])<<8 | uint32(buf[3])
	fmt.Printf("rawP: %x\n", rawP)
	rawT = uint32(buf[4])<<16 | uint32(buf[5])<<8 | uint32(buf[6])
	fmt.Printf("rawT: %x\n", rawT)
	return
}

func (s *Sensor) GetPressure() float64 {
	_, rawP, _, err := s.readRaw()
	if err != nil {
		panic(err)
	}
	return convertPressure(rawP)
}

func (s *Sensor) GetTemperature() float64 {
	_, _, rawT, err := s.readRaw()
	if err != nil {
		panic(err)
	}
	return convertTemperature(rawT)
}

func (s *Sensor) Close() error {
	return s.file.Close()
}

// implements datasheet formula: Y = 0.00286 * X - 18.75
func convertPressure(raw uint32) float64 {
	return 0.00286*float64(raw) - 18.75
}

// Placeholder until manufacturer provides a formula
func convertTemperature(raw uint32) float64 {
	return 0.00286*float64(raw) - 18.75
}
