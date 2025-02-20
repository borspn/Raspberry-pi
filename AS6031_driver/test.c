#include <stdio.h>
#include <pigpio.h>

#define PIN 24

void callback(int gpio, int level, uint32_t tick)
{
    printf("Interrupt on GPIO %d, Level: %d, Time: %u\n", gpio, level, tick);
}

int main()
{
    if (gpioInitialise() < 0)
    {
        printf("Failed to initialize GPIO!\n");
        return 1;
    }

    gpioSetMode(PIN, PI_INPUT);
    gpioSetPullUpDown(PIN, PI_PUD_UP);

    if (gpioSetISRFunc(PIN, EITHER_EDGE, 1000, callback) < 0)
    {
        printf("Failed to set ISR!\n");
        return 1;
    }

    printf("Waiting for interrupts...\n");
    while (1)
        sleep(1);

    gpioTerminate();
    return 0;
}
