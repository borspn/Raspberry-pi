package sensor

import (
	"fmt"
	"time"
)

var (
	chipName string = "gpiochip0"
	csGPIO   int    = 25
	spiDev   string = "spidev0.0"
) // change accordingly
func main() {

	sensor.InitSensor(dev, cs) // exported from sensor.go
	sensor.SensorInit()        // one-shot AS6031 setup

	ticker := time.NewTicker(10 * time.Second)
	defer ticker.Stop()
	for {
		// Read flow rate
		flowRate := sensor.ReadFlowRate()
		fmt.Printf("Flow Rate: %.6f \n", flowRate)
		// Wait for next tick
		<-ticker.C
	}
}
