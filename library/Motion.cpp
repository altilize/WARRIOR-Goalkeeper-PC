#include "Motion.h"
#include "Algoritma.h"
#include "TypeData.h"
#include <unistd.h>

#define pi (float)3.1428571428571428571

///////////////variable untuk hindar musuh/////////////////////

bool enemy=false;
bool zero=false;

//////////////////////////////////////////////////////////////
bool R2CMotion::pindahgrid(int x, int y, STM32Data& mSTM32Data,pcData& mPcData, int Vjauh,int Vdeket){

	int vx, vy;
	int kecepatan;
    int dx = x - mSTM32Data.GRIDX;
    int dy = y - mSTM32Data.GRIDY;
    int sudut = atan2((dx), (dy)) * 180 / pi;

    if (dx == 0 && dy == 0)
	{

		R2CMotion::Motion(mPcData,1,0,0,0,2);
		return true;
	}

	if (dx < 0)
	{
		if(dx < -2)
		{
			vx = Vjauh;
		}else
		{
			vx = Vdeket;
		}
	}	
	else if (dx > 0)
	{
		if( dx> 2)
		{
			vx = -1*Vjauh;
		}else
		{
			vx = -1*Vdeket;
		}
	}	
	else
	{
		vx = 0;
	}	

	if (dy < 0)
	{
		if(dy < -2)
		{
			vy = -1*Vjauh;
		} else
		{
			vy = -1*Vdeket;
		}
	}	
	else if (dy > 0)
	{
		if(dy > 2)
		{
			vy = Vjauh;
		}else
		{
			vy = Vdeket;
		}
	}	
	else
	{
		vy = 0;
	}

	kecepatan = sqrt((vx * vx) + (vy * vy));

    if(sudut >= 0 && sudut < 90){
		if(mSTM32Data.KOMPAS >= -5 && mSTM32Data.KOMPAS <= 5) mPcData.HEADING = sudut;
		else if(mSTM32Data.KOMPAS < (sudut - 180)) mPcData.HEADING = ((mSTM32Data.KOMPAS + 360) - sudut) * -1;
		else mPcData.HEADING = sudut - mSTM32Data.KOMPAS;	
	}

	else if(sudut >= 90 && sudut <= 180){
		if(mSTM32Data.KOMPAS >= -5 && mSTM32Data.KOMPAS <= 5) mPcData.HEADING = sudut;
		else if(mSTM32Data.KOMPAS < (sudut - 180)) mPcData.HEADING = ((mSTM32Data.KOMPAS + 90) + sudut) * -1;
		else mPcData.HEADING = sudut - mSTM32Data.KOMPAS;
	}

	else{
		if(mSTM32Data.KOMPAS >= -5 && mSTM32Data.KOMPAS <= 5) mPcData.HEADING = sudut;
		else if(mSTM32Data.KOMPAS >= (180 + sudut)) mPcData.HEADING = ((180 - mSTM32Data.KOMPAS) + (sudut + 180));
		else mPcData.HEADING = sudut - mSTM32Data.KOMPAS;
	}
	mPcData.MOTION=1;
	mPcData.KECEPATAN=kecepatan;
	mPcData.HANDLER = 2;
	//R2CMotion::cal
	int kp = R2CAlgoritma::calculateOutput(1,0,0, 1, mSTM32Data.KOMPAS*-1);
	mPcData.VZ = kp;
    return false;
}

int R2CMotion::Motion(pcData& mPcData,int motion, int heading, int kecepatan, int vz, int handler){
    mPcData.MOTION=motion;
    mPcData.HEADING=heading;
    mPcData.KECEPATAN=kecepatan;
    mPcData.VZ=vz;
    mPcData.HANDLER=handler;
	return true;
}

bool R2CMotion::setgrid(pcData& mPcData,int changegrid, int X, int Y, int kompas){
	mPcData.CHANGEGRID=changegrid;
	mPcData.GRIDX=X;
	mPcData.GRIDY=Y;
	if(kompas<0){
		mPcData.RESET_COMPASS_SIGN = true;
		mPcData.RESET_COMPASS=kompas*-1;
	}
	else{
		mPcData.RESET_COMPASS_SIGN = false;
		mPcData.RESET_COMPASS=kompas;
	}
	// printf("data sementara %i\n", mPcData.RESET_COMPASS);
	return true;
}

bool R2CMotion::putarrobotdarikompas(pcData& mPcData, STM32Data& mSTM32Data, int setpoint, int offset){
	double PID = R2CAlgoritma::calculateOutput(0.2,0,0,setpoint,mSTM32Data.KOMPAS)*-1;
	// printf("masukfungsi\n");
	if (PID > 10)
		PID = 9;
	else if (PID < -10)
		PID = -9;
    
	if ((mSTM32Data.KOMPAS<setpoint+offset)&&(mSTM32Data.KOMPAS>setpoint-offset))
	{
		mPcData.MOTION = 1;
		mPcData.HANDLER = 1;
		mPcData.KECEPATAN = 0;
		mPcData.HEADING = 0;
		mPcData.VZ = 0;
		return true;
	}

	else
	{
		mPcData.MOTION = 1;
		mPcData.HANDLER = 1;
		mPcData.KECEPATAN = 0;
		mPcData.HEADING = 0;
		mPcData.VZ = (PID /0.5) * 1.5; //3
	}
	return false;
}

bool R2CMotion::tendangkicker(pcData& mPcData, int power, bool enablekick){
	if (enablekick){
		// printf("masukif\n");
		mPcData.MOTION=1;
		mPcData.KICK_MODE=enablekick;
		mPcData.TENDANG=power;	
		return true;
		enablekick=false;
	}
	else {
		// printf("masukelse\n");
		mPcData.MOTION=1;
		mPcData.KICK_MODE=0;
		mPcData.TENDANG=0;
		// usleep(10000);
		return false;
	}
}


bool R2CMotion::hindarimusuh(pcData& mPcData, STM32Data& mSTM32Data){
	if(!enemy && !zero){
		R2CMotion::Motion(mPcData,1,180,100,0,2);
		usleep(50000);
		enemy=true;
		zero=true;
		// if(mSTM32Data.IR_MUSUH_1){

		// 	enemy=true;
		// 	zero=true;
		// }
	}
	if(enemy){
        mPcData.MOTION=1;
        mPcData.KECEPATAN=100;
        mPcData.HEADING=150;
        mPcData.VZ=70;
     	if(mSTM32Data.KOMPAS>175){
        	enemy=false;
            }
        }
    else if(!enemy && zero){
        if(R2CMotion::putarrobotdarikompas(mPcData,mSTM32Data,0,5)){
			R2CMotion::Motion(mPcData,1,0,0,0,2);
			enemy=false;
			zero=false;
			return true;
		}
    }
}
bool R2CMotion::movingrotation(float angle_bola, pcData& mPcData, STM32Data& mSTM32Data){
 	int heading = angle_bola;
	float kp;
	int heading_error = heading;
	if(heading_error<3 && heading_error>-3){
		kp = 3;
	}
	else{
		kp = 2;
	}
	mPcData.KECEPATAN = 100;//roda 3 100 roda 4 7
	mPcData.HEADING = heading_error;
	mPcData.VZ = kp*heading_error*-1;
	printf("%f ",kp);
	printf("%i ",heading_error);
	printf("%i\n",mPcData.VZ);
}
int konfirm1 = 0;
bool R2CMotion::hadap_teman(pcData& mPcData, STM32Data& mSTM32Data , int angle_teman, float anglebola, bool flagkawan,bool flagbola,bool flagnendang){
		float kp = 1;
		bool hit = false;
		// printf("konfirm: %i",konfirm1);
		
		if((angle_teman<=2 && angle_teman>=-2) && flagkawan == true){
			kp = 0;
			hit = true;
			konfirm1 ++;
		}
		// else if(angle_teman<-2){
		// 	kp = 50;
		// 	konfirm1 = 0;
		// }
		// else if(angle_teman>2){
		// 	kp = -50;
		// 	konfirm1 = 0;
		// }
		else{
			kp = R2CAlgoritma::calculateOutput(1,0,0.425,0,angle_teman);
			// kp = 100;
			konfirm1 = 0;
		}
		// else if(angle_teman<=3 && angle_teman>=-3){
		// 	kp = 2.7;
		// 	konfirm1 = 0;
		// }
		// else if(angle_teman<=5 && angle_teman>=-5){
		// 	kp = 0.9;
		// 	konfirm1= 0;
		// }
		// else if(angle_teman<=10 && angle_teman>=-10){
		// 	kp = 1;
		// 	konfirm1= 0;
		// }
		// else{
		// 	kp = 0.8;
		// 	konfirm1= 0;
		// }
		if (!flagbola)
		{
			mPcData.VZ = kp;
			if(flagkawan == true && hit == true){
				konfirm1= 0;
				return true;
			}
		}
		else if(flagbola){
			mPcData.VZ = kp;
			if(flagkawan == true && hit == true && flagnendang ==true){
				konfirm1= 0;
				mPcData.KICK_MODE = 1;
            	mPcData.TENDANG = 15;
            	usleep(60000);
            	mPcData.KICK_MODE = 0;
            	mPcData.TENDANG = 0;
				return true;
			}
			else{
				mPcData.HEADING = 0;
				mPcData.KECEPATAN = 0;
				return true;

			}
		}
	// printf("tes jadi\n");
	return false;

}
bool R2CMotion::putar_gawang(pcData& mPcData, STM32Data& mSTM32Data){
	if(mSTM32Data.BOLA){
		if(mSTM32Data.KOMPAS<25 || mSTM32Data.KOMPAS>35){
		mPcData.VZ = (-30+mSTM32Data.KOMPAS)*2;
	}
	else{
		// mPcData.KICK_MODE = 1;
        // mPcData.TENDANG = 11;
        // usleep(60000);
        // mPcData.KICK_MODE= 0;
        // mPcData.TENDANG = 0;
		// printf("tendang!!!");
		return true;
	}
	}
	else{
		mPcData.VZ = 0;
	}
	return false;
}

bool R2CMotion::mengitariobjek(float angle_bola, int headingtujuan,pcData& mPcData, STM32Data& mSTM32Data){
	int heading = angle_bola;
	float kp;
	int heading_error = heading;
	if(mSTM32Data.KOMPAS != headingtujuan){
		kp = R2CAlgoritma::calculateOutput(2.5,0,0.425,0,angle_bola);
	}
	else{
		kp = 0;
		return true;
	}

	mPcData.KECEPATAN = 80;//roda 3 100 roda 4 7
	mPcData.HEADING = 60;
	mPcData.VZ = kp;
	printf("kp: %f ",angle_bola);
	printf("%i\n",mPcData.VZ);
	return false;
}

bool R2CMotion::sampingkiper(pcData& mPcData, STM32Data& mSTM32Data){
	mPcData.HEADING = 90;
	
	mPcData.KECEPATAN = 100;
	int kp = R2CAlgoritma::calculateOutput(1,4,0,0,mSTM32Data.KOMPAS*-1);
	mPcData.VZ = kp;
	printf("%i\n",mPcData.VZ);
	return false;
}

// bool R2CMotion::pindahgrid2(int x, int y, STM32Data& mSTM32Data, pcData& mPcData, int Vjauh, int Vdeket) {
//     int dx = x - mSTM32Data.GRIDX;
//     int dy = y - mSTM32Data.GRIDY;
    
//     if (dx == 0 && dy == 0) {
//         R2CMotion::Motion(mPcData, 1, 0, 0, 0, 2);
//         return true;
//     }
    
//     int vx = (abs(dx) > 2) ? (dx > 0 ? -Vjauh : Vjauh) : (dx != 0 ? -Vdeket : 0);
//     int vy = (abs(dy) > 2) ? (dy > 0 ? Vjauh : -Vjauh) : (dy != 0 ? Vdeket : 0);
    
//     int kecepatan = sqrt((vx * vx) + (vy * vy));
//     float sudut = atan2(dx, dy) * 180 / pi; // Corrected atan2 parameters
    
//     // Normalize heading to reduce oscillations
//     float headingError = sudut - mSTM32Data.KOMPAS;
//     if (headingError > 180) headingError -= 360;
//     if (headingError < -180) headingError += 360;
    
//     if (abs(headingError) > 5) {
//         mPcData.HEADING = sudut;
//     }
    
//     mPcData.MOTION = 1;
//     mPcData.KECEPATAN = kecepatan;
//     mPcData.HANDLER = 2;
//     mPcData.VZ = 0;
    
//     return false;
// }

bool R2CMotion::pindahgrid2(int x, int y, STM32Data& mSTM32Data,pcData& mPcData, int Vjauh,int Vdeket,int targetsudut){

	int kp = 0;
	int vx, vy;
	int kecepatan;
    int dx = x - mSTM32Data.GRIDX;
    int dy = y - mSTM32Data.GRIDY;
    int sudut = atan2((dx), (dy)) * 180 / pi;
	// -------------------------- PID KOMPAS ----------------------------------------
	if(targetsudut != 0){
		kp = R2CAlgoritma::calculateOutput(1,0,0, targetsudut*-1,mSTM32Data.KOMPAS*-1);
		mPcData.VZ = kp;
		printf("%i\n",mPcData.VZ);
	}
	// ------------------------------------------------------------------------------
	else{
		kp = 0;
	}
    if (dx == 0 && dy == 0)
	{

		R2CMotion::Motion(mPcData,1,0,0,0,2);
		return true;
	}

	if (dx < 0)
	{
		if(dx < -2)
		{
			vx = Vjauh;
		}else
		{
			vx = Vdeket;
		}
	}	
	else if (dx > 0)
	{
		if( dx> 2)
		{
			vx = -1*Vjauh;
		}else
		{
			vx = -1*Vdeket;
		}
	}	
	else
	{
		vx = 0;
	}	

	if (dy < 0)
	{
		if(dy < -2)
		{
			vy = -1*Vjauh;
		} else
		{
			vy = -1*Vdeket;
		}
	}	
	else if (dy > 0)
	{
		if(dy > 2)
		{
			vy = Vjauh;
		}else
		{
			vy = Vdeket;
		}
	}	
	else
	{
		vy = 0;
	}

	kecepatan = sqrt((vx * vx) + (vy * vy));

    int delta = sudut - mSTM32Data.KOMPAS;

	// Normalize the delta to the -180 to 180 range
	if (delta > 180) delta -= 360;
	if (delta < -180) delta += 360;

	mPcData.HEADING = delta;
	mPcData.MOTION=1;
	mPcData.KECEPATAN=kecepatan;
	mPcData.HANDLER = 2;
	mPcData.VZ = kp;
    return false;
}
