void gpio_callback(int gpio, int level, uint32_t tick)
{
    printf("Interrupt detected on GPIO %d, level: %d, tick: %u\n", gpio, level, tick);
}

int main()
{
    if (gpioInitialise() < 0) 
    {
        printf("Failed to initialize pigpio!\n");
        return 1;
    }

    gpioSetMode(INTERRUPT_GPIO_PIN, PI_INPUT);
    gpioSetPullUpDown(INTERRUPT_GPIO_PIN, PI_PUD_UP);  // Set pull-up resistor if needed

    if (gpioSetAlertFunc(INTERRUPT_GPIO_PIN, gpio_callback) < 0)
    {
        printf("Failed to set alert function!\n");
        return 1;
    }

    while (1)
    {
        sleep(1);  // Keep the program running
    }

    gpioTerminate();
    return 0;
}
