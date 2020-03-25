# Makefile for the modbuspoll program.

# Use -g to compile the program for debugging.
DEBUG = -O2
#DEBUG = -g -DDEBUG
CFLAGS = $(DEBUG) -Wall -Werror -I/usr/local/include/modbus -L/usr/local/lib

LIBS = -lmodbus -lncurses


OFILES = modbuspoll.o

modbuspoll: $(OFILES)
	$(CC) $(CFLAGS) -o modbuspoll $(OFILES) $(LIBS)

modbuspoll.o:


.PHONY: clean
clean:
	rm -f $(OFILES) modbuspoll

