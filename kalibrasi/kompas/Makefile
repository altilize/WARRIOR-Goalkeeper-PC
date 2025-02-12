CC = g++
CFLAGS = -g -w -std=c++11

all: main

clean:
	rm -f *.o
	rm -f kalibrasikompas

KomunikasiArduino.o:
	$(info Compiling library Komunikasi Arduino)
	$(CC) $(CFLAGS) -c ../../lib/KomunikasiArduino.cpp

main: KomunikasiArduino.o
	$(info Compiling main program)
	$(CC) $(CFLAGS) KomunikasiArduino.o kalibrasikompas.cpp -lpthread -o kalibrasikompas
	
