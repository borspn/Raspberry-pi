package main

import (
	"fmt"
	"os"
	"syscall"
	"time"
)

const (
	I2C_SLAVE   = 0x0703
	MEASURE_CMD = 0xAA
	STATUS_OK   = 64
)

type PTSensor struct {
	delay       time.Duration
	ptDev       *os.File
	temperature float64
	pressure    float64
}

func (s *PTSensor) readRaw() (status byte, rawP uint32, rawT uint32, err error) {
	if _, err = s.ptDev.Write([]byte{MEASURE_CMD}); err != nil {
		return
	}
	time.Sleep(s.delay)
	buf := make([]byte, 1+3+3)
	if _, err = s.ptDev.Read(buf); err != nil {
		return
	}
	status = buf[0]

	rawP = uint32(buf[1])<<16 | uint32(buf[2])<<8 | uint32(buf[3])
	rawT = uint32(buf[4])<<16 | uint32(buf[5])<<8 | uint32(buf[6])

	return
}

// formula was provided by vendor (see verefication spreadsheet)
func convertPressure(raw uint32) float64 {
	return 0.000011175871*float64(raw) - 18.75
}

// formula was provided by vendor (see verefication spreadsheet)
func convertTemperature(raw uint32) float64 {
	return 0.00000983476639*float64(raw) - 40
}

func New(bus string, addr uint8, measureDelay time.Duration) (*PTSensor, error) {
	f, err := os.OpenFile("/dev/i2c-"+bus, os.O_RDWR, 0)
	if err != nil {
		return nil, err
	}
	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL, f.Fd(), I2C_SLAVE, uintptr(addr)); errno != 0 {
		f.Close()
		return nil, errno
	}
	return &PTSensor{ptDev: f, delay: measureDelay, temperature: 0.0, pressure: 0.0}, nil
}

func (s *PTSensor) GetPressure() float64 {
	return s.pressure
}

func (s *PTSensor) GetTemperature() float64 {
	return s.temperature
}

func (s *PTSensor) Close() error {
	return s.ptDev.Close()
}

func (sensor *PTSensor) Update() {
	status, rawP, rawT, err := sensor.readRaw()
	if err != nil {
		fmt.Println("Error reading sensor data:", err)
		return
	}
	if status != STATUS_OK {
		fmt.Println("PTSensor status error:", status)
		return
	}
	sensor.pressure = convertPressure(rawP)
	sensor.temperature = convertTemperature(rawT)
	fmt.Printf("Updated Temperature: %.2f °C, Pressure: %.2f Pa\n", sensor.temperature, sensor.pressure)
}
