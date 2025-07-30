package main

import (
	"fmt"
	"time"
)

// flow sensor
const (
	CHIP_NAME string = "gpiochip4"
	CS_GPIO   int    = 9
	SPI_DEV   string = "spidev0.0"
)

// PT sensor
const (
	PT_SLAVE_ADDR uint8         = 0x00
	MEAS_DELAY    time.Duration = 10 * time.Millisecond
	BUS           string        = "1"
)

func runFlowTest(chipName string, csGPIO int, spiDev string) {
	InitSPI(chipName, csGPIO, spiDev)
	SensorInit()
	t := time.NewTicker(10 * time.Second)
	for fr := range t.C {
		_ = fr // just wait
		fmt.Printf("flow: %.6f\n", ReadFlowRate())
	}
}

func runPtTest(slave_addr uint8, meas_delay time.Duration, bus string) {
	s, err := New(bus, slave_addr, meas_delay)
	if err != nil {
		panic(err)
	}
	defer s.Close()
	s.Update()
	fmt.Printf("Temperature: %.2f °C\n", s.GetTemperature())
	fmt.Printf("Pressure:    %.2f Pa\n", s.GetPressure())
}

func main() {
	runFlowTest(CHIP_NAME, CS_GPIO, SPI_DEV)
	runPtTest(PT_SLAVE_ADDR, MEAS_DELAY, BUS)
}
