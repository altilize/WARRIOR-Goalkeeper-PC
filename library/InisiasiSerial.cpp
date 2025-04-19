#include "InisiasiSerial.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>



int Serial::getindexportSTM32(const char *port) 
{
    index = open(port, O_RDWR | O_NOCTTY);
    if (index ==-1){
        printf("Port %s tidak dapat dibuka\n", port);
    }
    return index;
}

int Serial::errorattributeterminal()
{
    if (tcgetattr(index, &toptions) != 0) {
        perror("Error getting terminal attributes");
        return 1;
    }
}

void Serial::initserial()
{
    tcgetattr(index, &toptions);

	cfsetispeed(&toptions, 115200); //baudRate
	cfsetospeed(&toptions, 115200);

	toptions.c_cflag &= ~PARENB; // no parity
	toptions.c_cflag &= ~CSTOPB; // 1 stop bit
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8; // 8 data bits

	toptions.c_cflag &= ~CRTSCTS;

	toptions.c_cflag |= CREAD | CLOCAL;
	// toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
	toptions.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	toptions.c_oflag &= ~OPOST;

	toptions.c_cc[VMIN] = 12; //0; //PANJANG_DATA_ARDUINO;
	toptions.c_cc[VTIME] = 0; //10;

	// menerapkan pengaturan
	tcsetattr(index, TCSANOW, &toptions);

	tcflush(index, TCIFLUSH);
}

int Serial::readserial(unsigned char *hasil){
    tcflush(index, TCIFLUSH);
	int buffSize = read(index, hasil, 128); //PANJANG_DATA_ARDUINO);
	// printf("%i\n",buffSize);
	return buffSize;
}

int Serial::writeSerial(unsigned char *text, int size)
{
	// printf("masukserial\n");
	return write(index, text, size);
}

