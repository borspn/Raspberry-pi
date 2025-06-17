package main

import (
	"fmt"
	//"time"
)

var (
	chipName string = "gpiochip0"
	csGPIO   int    = 25
	spiDev   string = "spidev0.0"
) // change accordingly

func main() {
	InitSPI(chipName, csGPIO, spiDev)
	writeOpcode(99)
	writeDword(rcRAAWRRAM, byte(shrEXC), (fesCLRMask | efCLRMask | ifCLRMask))
	readDword(rcRAARDRAM, srrERRFLAG)
	SensorInit()
	fmt.Println("success")
	//SensorInit()

	// t := time.NewTicker(10 * time.Second)
	// for fr := range t.C {
	// 	_ = fr // just wait
	// 	fmt.Printf("flow: %.6f\n", ReadFlowRate())
	// }
}
