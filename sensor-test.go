package main

import (
	"flag"
	"fmt"
	"time"
)

// ─── Flow‑sensor constants ──────────────────────────────────────────────────────
const (
	CHIP_NAME string = "gpiochip4"
	CS_GPIO   int    = 9
	SPI_DEV   string = "spidev0.0"
)

// ─── PT‑sensor constants ────────────────────────────────────────────────────────
const (
	PT_SLAVE_ADDR uint8         = 0x00
	MEAS_DELAY    time.Duration = 10 * time.Millisecond
	BUS           string        = "1"
)

func runFlowTest(chipName string, csGPIO int, spiDev string) {
	InitSPI(chipName, csGPIO, spiDev)
	SensorInit()
	t := time.NewTicker(2 * time.Second)
	defer t.Stop()
	for i := 0; i < 3; i++ {
		<-t.C
		fmt.Printf("flow: %.6f\n", ReadFlowRate())
	}
}

func runPtTest(slaveAddr uint8, measDelay time.Duration, bus string) {
	s, err := New(bus, slaveAddr, measDelay)
	if err != nil {
		panic(err)
	}
	defer s.Close()
	s.Update()
	fmt.Printf("Temperature: %.2f °C\n", s.GetTemperature())
	fmt.Printf("Pressure:    %.2f Pa\n", s.GetPressure())
}

func main() {
	// ─── Command‑line flags ────────────────────────────────────────────────────
	fFlag := flag.Bool("f", false, "run flow‑sensor test only")
	pFlag := flag.Bool("p", false, "run PT‑sensor test only")
	flag.Parse()

	// ─── Dispatch ──────────────────────────────────────────────────────────────
	switch {
	case *fFlag && !*pFlag: // ‑f only
		runFlowTest(CHIP_NAME, CS_GPIO, SPI_DEV)
	case *pFlag && !*fFlag: // ‑p only
		runPtTest(PT_SLAVE_ADDR, MEAS_DELAY, BUS)
	default: // neither flag or both flags → run both
		runFlowTest(CHIP_NAME, CS_GPIO, SPI_DEV)
		runPtTest(PT_SLAVE_ADDR, MEAS_DELAY, BUS)
	}
}
