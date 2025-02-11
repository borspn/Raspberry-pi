CC=gcc
CFLAGS=-Wall -I.
LDFLAGS=-L/usr/local/lib -lwiringPi

SRCS=main.c SPI_interface.c WiringPi/wiringPi/wiringPi.c
OBJS=$(SRCS:.c=.o)
TARGET=as6031_driver

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
