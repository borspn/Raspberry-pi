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
	InitSPI(chipName, csGPIO, spiDev)
	// Read and print flow rate every 10 seconds
	ticker := time.NewTicker(10 * time.Second)
	defer ticker.Stop()
	for {
		// Read flow rate
		flowRate := ReadFlowRate()
		fmt.Printf("Flow Rate: %.6f \n", flowRate)
		// Wait for next tick
		<-ticker.C
	}
}
