# Makefile for compiling SPI_interface.c and main.c with libgpiod library

# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -O2 -I./scioSenseLib -I./spi

# Include directories
INCLUDES = -I/usr/local/include

# Library directories
LIBDIRS = -L/usr/local/lib

# Libraries to link against
LIBS = -lgpiod -lpthread

# Source files
SRCS = main.c spi/SPI_interface.c

# Object files
OBJS = $(SRCS:.c=.o)

# Executable name
TARGET = as6031_driver

# Default target
all: $(TARGET)

# Rule to build the executable
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(LIBDIRS) -o $(TARGET) $(OBJS) $(LIBS)

# Rule to compile source files into object files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
