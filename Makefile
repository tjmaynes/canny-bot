# Author: TJ Maynes
# File: Makefile

CFLAGS = -Wall -o
PROGRAM = main

main:
	g++ CannyBot.cpp $(CFLAGS) $(PROGRAM)
	./main

clean:
	rm -f $(PROGRAM)
