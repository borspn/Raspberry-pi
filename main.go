package main

import (
	"fmt"
	"time"

	"RaspPi/sensor"
)

func main() {
    sensor.InitSensor("spidev0.0", 25)
    sensor.SensorInit()

    t := time.NewTicker(10 * time.Second)
    for fr := range t.C {
        _ = fr // just wait
        fmt.Printf("flow: %.6f\n", sensor.ReadFlowRate())
    }
}
