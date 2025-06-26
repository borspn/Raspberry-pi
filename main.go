package main

import (
	"fmt"
	"time"
)

func main() {
	// bus "1" → /dev/i2c-1, sensor at 0x76, needs ~10 ms per measure
	s, err := New("1", 0x00, 10*time.Millisecond)
	if err != nil {
		panic(err)
	}
	defer s.Close()

	fmt.Printf("Temperature: %.2f °C\n", s.GetTemperature())
	fmt.Printf("Pressure:    %.2f Pa\n", s.GetPressure())
}
