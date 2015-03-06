# Author: TJ Maynes
# File: Makefile

CFLAGS = -Wall -o
PROGRAM = main

main:
	python CannyBot.py

old:
	g++ CannyBot.cpp $(CFLAGS) $(PROGRAM)
	./main

clean:
	rm -f $(PROGRAM)
