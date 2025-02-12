#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <math.h>
#include <vector>
#include "../../lib/KomunikasiArduino.h"
#include "../../lib/Particlefilter.h"
#include "../../lib/TipeData.h"

#define detik 1
#define gridSize (float) 10
#define pi (float) 3.1428571428571428571

using namespace std;

float arahGawangLawan = 0;
int heading = 0;

Serial arduino;
arduinoData mArduinoData;
unsigned char dataMasukArduino[100];
int lengthDataArduino=0;
pcData mPcData;

vector<mouse_pos> dataMouseX;
vector<mouse_pos> dataMouseY;

mouse_pos current_mouse;
bool showKompas = false;

// Digunanakn untuk sorting vector
struct less_than_key
{
	inline bool operator()(const mouse_pos& struct1, const mouse_pos& struct2){
		return struct1.r < struct2.r;
	}
};

void init_main()
{
	R2CKomunikasiArduino::init_arduino(arduino);

	mPcData.MOTION = 0;
	mPcData.HEADING = 0;
	mPcData.KECEPATAN = 0;
	mPcData.TENDANG = 0;
	mPcData.HANDLER = 0;
	mPcData.GRIDX = 0;
	mPcData.GRIDY = 0;
	mPcData.HEADINGPRINT = 0;
	mPcData.APP = 1;
	heading = 0;

	FILE* input = fopen("/home/r2c/war/kalibrasi/kompas.txt","r");
	if(input == NULL)
	{
		printf("File kompas tidak ada\n");
	}
	fscanf(input,"%f\n",&arahGawangLawan);
	fclose(input);
}

void thrd_komunikasiArdu()
{
	while(1)
	{
		lengthDataArduino = arduino.readSerial(dataMasukArduino);

		// R2CKomunikasiArduino::printDataArduino(dataMasukArduino);

		if(R2CKomunikasiArduino::parseArduinoData(mArduinoData,lengthDataArduino,dataMasukArduino))
		{

		}
		R2CKomunikasiArduino::convert2heading(mArduinoData.KOMPAS, arahGawangLawan, heading);

		if(showKompas)
		{
			printf("%f  || %f  ||  %i   \n", mArduinoData.KOMPAS, arahGawangLawan, heading);
		}

		if(R2CKomunikasiArduino::kirimParamAlgo(arduino, mPcData, heading) == -1)
		{
			R2CKomunikasiArduino::init_arduino(arduino);
			continue;
		}

		current_mouse.mouse_x += (cos(heading * pi / 180) * mArduinoData.deltaX) + (cos((heading+90)*pi/180) * mArduinoData.deltaY);
		current_mouse.mouse_y += (sin(heading * pi / 180) * mArduinoData.deltaX) + (sin((heading+90)*pi/180) * mArduinoData.deltaY);

//		current_mouse.mouse_x += ardu1.deltaX;
//		current_mouse.mouse_y += ardu1.deltaY;
	}
}

void thrd_menu()
{
	int input = -1;
	float jumlahX = 0, jumlahY = 0;
	int totalX = 0, totalY = 0;
	int kecepatan = 70;
	int waktu = 3;

	while(input != 0)
	{
		system("clear");
		jumlahX = 0;
		jumlahY = 0;
		printf("\nKecepatan: %i\nWaktu: %i detik\n \t x \t y \t r \t panjang\n", kecepatan, waktu);
		for(int i=0;i<dataMouseX.size();i++){
			long int nilaiR = sqrt((dataMouseX[i].mouse_x*dataMouseX[i].mouse_x)+(dataMouseX[i].mouse_y*dataMouseX[i].mouse_y));
			dataMouseX[i].r = nilaiR;
		}
		for(int i=0;i<dataMouseY.size();i++){
			long int nilaiR = sqrt((dataMouseY[i].mouse_x*dataMouseY[i].mouse_x)+(dataMouseY[i].mouse_y*dataMouseY[i].mouse_y));
			dataMouseY[i].r = nilaiR;
		}

		for(int j=0;j<dataMouseX.size();j++){
			std::sort(dataMouseX.begin()+j, dataMouseX.end(), less_than_key());
		}
		for(int j=0;j<dataMouseY.size();j++){
			std::sort(dataMouseY.begin()+j, dataMouseY.end(), less_than_key());
		}

		printf("\nX:\n");
		for(int i=0;i<dataMouseX.size();i++){
			printf("%i \t %li \t %li \t %li \t %li \n", i, dataMouseX[i].mouse_x, dataMouseX[i].mouse_y, dataMouseX[i].r, dataMouseX[i].panjang);
			jumlahX+=(((float)dataMouseX[i].r*gridSize)/dataMouseX[i].panjang);
		}
		if(dataMouseX.size()>0){
			totalX = jumlahX/dataMouseX.size();
			printf("nilai: %i\n", totalX);
		}

		printf("\nY:\n");
		for(int i=0;i<dataMouseY.size();i++){
			printf("%i \t %li \t %li \t %li \t %li \n", i, dataMouseY[i].mouse_x, dataMouseY[i].mouse_y, dataMouseY[i].r, dataMouseY[i].panjang);
			jumlahY+=(((float)dataMouseY[i].r*gridSize)/dataMouseY[i].panjang);
		}
		if(dataMouseY.size()>0){
			totalY = jumlahY/dataMouseY.size();
			printf("nilai: %i\n", totalY);
		}

		printf("\n\n");
		printf("1. Kalibrasi X\n2. Kalibrasi Y\n3. Hapus data X\n4. Hapus data Y\n5. tampilkan kompas\n6. Set kecepatan\n7. Set Waktu\n0. Keluar dan save\n");
		scanf("%i", &input);

		if(input==1 || input==2)
		{
			R2CParticleFilter::resetMotionModel();
			// algoritmaMain.motion = 3;
			// algoritmaMain.heading = 0;
			// algoritmaMain.kecepatan = kecepatan;

			current_mouse.mouse_x = 0;
			current_mouse.mouse_y = 0;
			usleep(waktu*1000000);

			// algoritmaMain.motion = 0;
			// algoritmaMain.heading = 0;
			// algoritmaMain.kecepatan = 0;

			usleep(1000*1000);
			mouse_pos mouse_save;
			mouse_save.mouse_x = current_mouse.mouse_x;
			mouse_save.mouse_y = current_mouse.mouse_y;
			printf("x: %li | y: %li\n", current_mouse.mouse_x, current_mouse.mouse_y);

			int input2;
			printf("Masukkan panjang pergerakan: \n");
			scanf("%i", &input2);
			mouse_save.panjang = input2;

			if(input==1)
			{
				dataMouseX.push_back(mouse_save);
			}
			else if(input==2)
			{
				dataMouseY.push_back(mouse_save);
				printf("%i\n", dataMouseY.size());
			}
			input = -1;
		}else if(input==3 || input==4){
			printf("Masukkan index: ");
			scanf("%i", &input);
			if(input==3)
			{
				dataMouseX.erase(dataMouseX.begin()+input);
			}
			else
			{
				dataMouseY.erase(dataMouseY.begin()+input);
			}
			printf("\n\n\n");
			input = -1;
		}else if(input==0){
			break;
		}else if(input==5)
		{
			int ab = 0;
			showKompas = true;
			scanf("%i",&ab );
			showKompas = false;
			input = -1;
		}
		else if(input==6)
		{
			printf("Masukkan kecepatan: ");
			scanf("%i", &kecepatan);
		}
		else if(input==7)
		{
			printf("Masukkan waktu: ");
			scanf("%i", &waktu);
		}
	}
	FILE* kalMouse = fopen("/home/r2c/war/kalibrasi/mouse.txt", "w");
	fprintf(kalMouse, "%i\n%i\n", totalX, totalY);
	fclose(kalMouse);
	exit(0);
}

int main()
{
	printf("init\n");
	init_main();
	thread komunikasi(thrd_komunikasiArdu);
	thread menu(thrd_menu);

	komunikasi.join();
	menu.join();

	return 0;	
}
