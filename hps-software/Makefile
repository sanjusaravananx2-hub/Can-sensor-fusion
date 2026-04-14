# Makefile for HPS CAN Monitor Application
# Cross-compile: make CROSS_COMPILE=arm-linux-gnueabihf-
# Native (on DE1-SoC): make

CROSS_COMPILE ?=
CC = $(CROSS_COMPILE)gcc
CFLAGS = -Wall -Wextra -O2
TARGET = can_monitor

.PHONY: all clean

all: $(TARGET)

$(TARGET): can_monitor.c fpga_regs.h
	$(CC) $(CFLAGS) -o $@ can_monitor.c

clean:
	rm -f $(TARGET)
