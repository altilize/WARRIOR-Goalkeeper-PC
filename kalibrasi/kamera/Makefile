CC = g++
CFLAGS = -g -w -std=c++11 
OPENCV_FLAGS = `pkg-config --cflags --libs opencv4`

all: clean main

clean:
	$(info Cleaning files)
	rm -f deteksi_gawang

main:
	$(info Compiling main program)
	$(CC) $(CFLAGS) deteksi_gawang.cpp -o deteksi_gawang $(OPENCV_FLAGS)
