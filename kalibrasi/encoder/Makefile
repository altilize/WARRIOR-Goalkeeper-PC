CC = g++
CFLAGS = -g -w -std=c++11

all: main

clean:
	rm -f *.o
	rm -f kalibrasiencoder

KomunikasiArduino.o:
	$(info Compiling library Komunikasi Arduino)
	$(CC) $(CFLAGS) -c ../../lib/KomunikasiArduino.cpp

Particlefilter.o:
	$(info Compiling library Komunikasi Arduino)
	$(CC) $(CFLAGS) -c ../../lib/Particlefilter.cpp

main: KomunikasiArduino.o Particlefilter.o
	$(info Compiling main program)
	$(CC) $(CFLAGS) KomunikasiArduino.o Particlefilter.o kalibrasiencoder.cpp -lpthread -o kalibrasiencoder
	
