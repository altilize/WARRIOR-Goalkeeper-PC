#include "KomunikasiMikro.h"
#include "InisiasiSerial.h"
#include <unistd.h>
#include <stdio.h>
#include <iostream>

char USBSTM32[] = {"/dev/ttyACM0"};
int total_checksum = 0;
unsigned char dataaaserial[20];

// =============== Inisisasi Komunikasi =============
void R2CKomunikasiSTM32::STM32init(Serial &STM32){

	int abc = STM32.getindexportSTM32(USBSTM32);
	while (abc == -1)
	{
		abc = STM32.getindexportSTM32(USBSTM32);
		usleep(100 * 1000);
	}

	STM32.initserial();
	usleep(10 * 1000);
}

// ==================== Send Data to STM32 ================ //
int R2CKomunikasiSTM32::sendData(Serial com1, pcData dataPC)
{
	
	total_checksum = 0;
	
	if (dataPC.CHANGEGRID)
	{
		printf("Kirim GRID|%i\n",dataPC.RESET_COMPASS);
		dataaaserial[0] = 250;
		dataaaserial[1] = dataPC.GRIDX;
		dataaaserial[2] = dataPC.GRIDY;
		dataaaserial[3] = dataPC.RESET_COMPASS;
		dataaaserial[4] = dataPC.RESET_COMPASS_SIGN;
		dataaaserial[5] = 0;
		dataaaserial[6] = 0;
		dataaaserial[7] = 0;
		dataaaserial[8] = 0;
		dataaaserial[9] = 0;
		dataaaserial[10] = 0;
		dataaaserial[11] = 0;
		printf("datakompass reset %i\n",dataPC.RESET_COMPASS);
		dataPC.CHANGEGRID = false;
		dataPC.RESET_COMPASS = 0;
		printf("Selesai Kirim GRID HHH|%i\n",dataPC.CHANGEGRID);
	}
	else
	{
		dataaaserial[0] = 254;		
		if (dataPC.MOTION == 1) dataaaserial[1] = 1;
		else if (dataPC.MOTION == 0) dataaaserial[1] = 0;
		dataaaserial[2] = dataPC.KECEPATAN>>8; 
		dataaaserial[3] = dataPC.KECEPATAN; 
		// ------ handle heading and sign heading ------- //
		if (dataPC.HEADING < 0)
		{
			dataaaserial[4] = 1; 
			dataaaserial[5] = dataPC.HEADING*-1; 
		}
		else
		{
			dataaaserial[4] = 0; 
			dataaaserial[5] = dataPC.HEADING; 
		}
		// --------------- handle vz ------------------- //
		int vz = (int)dataPC.VZ;
		if (vz < 0)
		{
			vz *= -1;
			dataaaserial[6] = 1;
		}
		else{
		dataaaserial[6] = 0;
		}
		dataaaserial[7] = vz >> 8;
		dataaaserial[8] = vz; 
		// --------------------------------------------- //
		dataaaserial[9] = dataPC.HANDLER;
		dataaaserial[10] = dataPC.TENDANG;
		dataaaserial[11] = dataPC.KICK_MODE;
		dataaaserial[12] = dataPC.VX;
		dataaaserial[13] = dataPC.VY;
	}

	for (int i = 1; i < 12; i++)
	{
		total_checksum += dataaaserial[i];
		
	}
	dataaaserial[12] = ~total_checksum;
	return com1.writeSerial(dataaaserial, 13); 

}


// ============================= Read Data From STM32 =================================== //
bool R2CKomunikasiSTM32::parseSTM32Data(STM32Data &sData, const unsigned char *data)
{
	sData.HEADER = data[0];
	if(data[0] == 255){
		sData.KOMPAS = data[2];;
		if (data[1] == 1) sData.KOMPAS *= -1;
		sData.BUTTON = data[3];
		sData.linear= data[4];
		sData.IR_MUSUH_2 = data[5];
		sData.IR_MUSUH_3 = data[6];
		sData.BOLA  = data[7];
		sData.GRIDX = data[8];
		sData.GRIDY = data[9];

		sData.XPOS = (float)((data[11] << 8) + data[12]);

		if(data[10] == 1){ 
			sData.XPOS *= -1;
		}
		
		sData.YPOS = (float)((data[14] << 8) + data[15]);
		if(data[13] == 1) {
			sData.YPOS *= -1;
		} 
		// printf("%i|%2f\n", sData.IR_MUSUH_1, sData.YPOS);

	// 		for(int i=0;i<16;i++){
	// 	printf("%i|",data[i]);
	// }
	// printf("\n");
	
		return true;
	}
	
	
}
