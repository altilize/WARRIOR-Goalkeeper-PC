#include <stdio.h>
#include <unistd.h>
#include "../../lib/KomunikasiArduino.h"

int main()
{
	arduinoData mArduinoData;
	unsigned char dataMasukArduino[100];
	
	int lengthDataArduino;
	Serial arduino;
	float sum = 0;
	float nilaiKompas = 0;
	
	pcData mPcData;
	mPcData.MOTION = 0;
	mPcData.HEADING = 0;
	mPcData.KECEPATAN = 0;
	mPcData.TENDANG = 0;
	mPcData.HANDLER = 0;
	mPcData.GRIDX = 0;
	mPcData.GRIDY = 0;
	mPcData.HEADINGPRINT = 0;
	mPcData.APP = 1;
	int heading = 0;

	R2CKomunikasiArduino::init_arduino(arduino);

	printf("Start: Loop Komunikasi Serial---\n");	
	int index = 0;
	R2CKomunikasiArduino::kirimParamAlgo(arduino, mPcData, heading);

	while(index<13)
	{
		lengthDataArduino = arduino.readSerial(dataMasukArduino);
		
		R2CKomunikasiArduino::parseArduinoData(mArduinoData,lengthDataArduino,dataMasukArduino);
		printf("%.2f\n", mArduinoData.KOMPAS);
		sum += mArduinoData.KOMPAS;

		if(R2CKomunikasiArduino::kirimParamAlgo(arduino, mPcData, heading) == -1)
		{
			R2CKomunikasiArduino::init_arduino(arduino);
			continue;
		}
		index++;
	}

	nilaiKompas = sum/13;
	FILE* output = fopen("/home/r2c/war/kalibrasi/kompas.txt","w");
	if(output == NULL)
	{
		printf("file kompas tidak ada\n");
		return 0;
	}
	printf("\n\nNilai kompas: %f\n", nilaiKompas);
	fprintf(output,"%f\n",nilaiKompas);
	fclose(output);

	arduino.closeSerial();
	return 0;
}
