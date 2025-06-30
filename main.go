package main

import (
	"fmt"
	"time"
)

const (
	SLAVE_ADDR = 0x00
	MEAS_DELAY = 10 * time.Millisecond
	BUS        = "1" // I2C bus number, e.g., "1" for /dev/i2c-1
)

func main() {
	// bus "1" → /dev/i2c-1, sensor at 0x76, needs ~10 ms per measure
	s, err := New(BUS, SLAVE_ADDR, MEAS_DELAY)
	if err != nil {
		panic(err)
	}
	defer s.Close()
	s.Update()
	fmt.Printf("Temperature: %.2f °C\n", s.GetTemperature())
	fmt.Printf("Pressure:    %.2f Pa\n", s.GetPressure())
}
