CC=gcc
CFLAGS=-Wall -I.
LDFLAGS=-L/usr/local/lib

SRCS=main.c SPI_interface.c wiringPi.c
OBJS=$(SRCS:.c=.o)
TARGET=as6031_driver

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET) $(LIBS)

%.o: %.c %.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
