# Makefile for compiling SPI_interface.c and main.c with pigpio library

# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -O2

# Include directories
INCLUDES = -I/usr/local/include

# Library directories
LIBDIRS = -L/usr/local/lib

# Libraries to link against
LIBS = -lpigpio -lpthread

# Source files
SRCS = SPI_interface.c main.c

# Object files
OBJS = $(SRCS:.c=.o)

# Executable name
TARGET = spi_program

# Default target
all: $(TARGET)

# Rule to build the executable
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) $(LIBDIRS) -o $(TARGET) $(OBJS) $(LIBS)

# Rule to compile source files into object files
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
