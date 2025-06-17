package main

import (
	"fmt"
	"time"
)

func main() {
	InitSensor("spidev0.0", 25)
	SensorInit()

	t := time.NewTicker(10 * time.Second)
	for fr := range t.C {
		_ = fr // just wait
		fmt.Printf("flow: %.6f\n", ReadFlowRate())
	}
}
