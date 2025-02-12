#include "Algoritma.h"
#include "TypeData.h"
#include "Motion.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

#define KP (float)0.6//0.4//1.8 //3;
#define KI (float)0
#define KD (float)0.2//4 //1.0;

#define KP_gawang (float)0.2
#define KI_gawang (float)0.0
#define KD_gawang (float)0.2

int counter_rem_dekati_bola = 0;

float setpoint;
float processVariable;
float error_;
float previousError = 0;
float integral_ = 0;
float derivative_;
float integralLimit = 50;
bool flag_tendang;
bool flag_stop;
int count_gawang = 0;int counter=0;
bool maju=false;
bool caribola=false;
int error_sudut_bola,last_error_sudut_bola;
float propotional_sudut = 0.0;
float derivative_sudut = 0.0;
float integratif_sudut = 0.0;
float PD = 0.0;

double R2CAlgoritma::calculateOutput(float kp, float ki, float kd ,double setpoint, double processVariable){

    double error = setpoint - processVariable;
    integral_ += error;

    // Anti-Windup: Batasi nilai integral
    if (integral_ > integralLimit) {
        integral_ = integralLimit;
    } else if (integral_ < -integralLimit) {
        integral_ = -integralLimit;
    }

    double derivative = error - previousError;
    double output = kp * error + ki * integral_ + kd * derivative;
    previousError = error;

    return output;
}

bool R2CAlgoritma::lihat_bola_diam(bool param_bola, int angle_bola, pcData& mPcData, STM32Data& mSTM32Data)
{
    int counter = 0;
    float kp = 0.6;
    float ki = 0.0;
    float kd = 0.2;
    double setpoint = 0.0;
    double current_value = angle_bola;
    double kec_rotasi = R2CAlgoritma::calculateOutput(kp,ki,kd,0, angle_bola);

    if(kec_rotasi >= 10)
        {kec_rotasi = 10;}
    else if(kec_rotasi <= -10)
        {kec_rotasi = -10;}
    //printf("%f\n",kec_rotasi);
    
    if (param_bola)
    {
        kec_rotasi--;
        //cout << kec_rotasi << endl;
        if (angle_bola > 5)
        {
            mPcData.MOTION = 1;
            mPcData.KECEPATAN = 0;
            mPcData.HEADING = 0;
            mPcData.VZ = kec_rotasi/0.5;
            mPcData.HANDLER = 1;
            return false;
        }

        else if (angle_bola < -5)
        {
            mPcData.MOTION = 1;
            mPcData.KECEPATAN = 0;
            mPcData.HEADING = 0;
            mPcData.VZ = kec_rotasi/0.5;
            mPcData.HANDLER = 1;
            return false;
        }
        else if (angle_bola >= -5 && angle_bola <= 5)
        {
            mPcData.MOTION = 0;
            if (abs(angle_bola) < 5)
            {
                mPcData.VZ = 0;
                if(abs(angle_bola) < 4)
                {
                    mPcData.VZ = 0;
                    if (abs(angle_bola) < 3)
                    {
                        mPcData.VZ = 0;
                    }
                }

            }
            mPcData.VZ = 0;
            return true;
        }
    }
    else{
        mPcData.MOTION = 0;
        mPcData.VZ = 0;
        return false;
    }
}


bool R2CAlgoritma::lihat_bola_geser(int e_distance,int angle_bola, pcData& mPcData, STM32Data& mSTM32Data)
{
    double PID = calculateOutput(0.6, 0.0, 0.2, 0, angle_bola);
    if (PID > 9)
		PID = 9;
	else if (PID < -9)
		PID = -9;
    
    int kecepatan= PID;
    if(kecepatan<0){
        kecepatan*=-1;
    }
    else {
        kecepatan=kecepatan;
    }
    //printf("%f|%i|%i\n",PID,kecepatan,e_distance); 
    // if(e_distance<70){   
    if (angle_bola>3){
        R2CMotion::Motion(mPcData,1,90,kecepatan,0,2);
        return false;
    }
    else if(angle_bola<-3){
        R2CMotion::Motion(mPcData,1,-90,kecepatan,0,2);
        return false;
    }
    else if(angle_bola==0){
        R2CMotion::Motion(mPcData,1,0,0,0,2);
        return false;
    }
    else{
        R2CMotion::Motion(mPcData,0,0,0,0,0);
        return true;
    }
    
}


bool R2CAlgoritma::putar_cari_gawang(bool& is_goal_visible, bool& state_tiang, int& sudutGawang, int& sudutTiangKiri, int& sudutTiangKanan, pcData& mPcData, STM32Data& mSTM32Data){
    //cout << is_goal_visible << "|" << lebarGawang << "|" << (int)jarak_gawang_robot << "|" << sudutGawang << "|" << sudutGawangKiri << "|" << sudutGawangKanan << "|" << mSTM32Data.BOLA << endl;
    if (is_goal_visible == false)
    {
        flag_tendang = false;
    }
    else
    {
        flag_tendang = true;
    }

    if (flag_tendang == false)
    {
        mPcData.MOTION = 1;
        mPcData.KECEPATAN = 0;
        mPcData.HANDLER = 0;
        mPcData.HEADING = 0;
        mPcData.VZ = -2;
        mPcData.KICK_MODE = 0;
        mPcData.TENDANG = 0;
    }
    if (flag_tendang == true)
    {
        if (sudutTiangKanan == 0 && sudutTiangKiri == 0)
        {
            do{
                if (sudutGawang < -9)
                {
                    mPcData.MOTION = 1;
                    mPcData.KECEPATAN = 0;
                    mPcData.HANDLER = 1;
                    mPcData.HEADING = 0;
                    mPcData.VZ = -1;
                    mPcData.KICK_MODE = 0;
                    mPcData.TENDANG = 0;
                }
                if (sudutGawang > 9)
                {
                    mPcData.MOTION = 1;
                    mPcData.KECEPATAN = 0;
                    mPcData.HANDLER = 1;
                    mPcData.HEADING = 0;
                    mPcData.VZ = 1;
                    mPcData.KICK_MODE = 0;
                    mPcData.TENDANG = 0;   
                }
                if (sudutGawang > -9 && sudutGawang < 9)
                {
                    mPcData.MOTION = 1;
                    mPcData.KECEPATAN = 0;
                    mPcData.HANDLER = 1;
                    mPcData.HEADING = 0;
                    mPcData.VZ = 0;
                    mPcData.KICK_MODE = 0;
                    mPcData.TENDANG = 0;   
                }  
                count_gawang++;
                cout << count_gawang << "|" << flag_tendang << endl;
            }while (count_gawang < 2000);
            
            if (count_gawang > 2000)
            {
                mPcData.MOTION = 0;
                mPcData.KECEPATAN = 0;
                mPcData.HANDLER = 0;
                mPcData.HEADING = 0;
                mPcData.VZ = 0; 
                mPcData.KICK_MODE = 1;
                mPcData.TENDANG = 10;
                usleep(3000);  
                count_gawang = 0;
                return true;
            }

        }
    }
}

int Vz_gawang = 7;
int angle_thresh = 2;
float scale = 2;//2 //(1.7*3)
int power_kick = 8;

bool R2CAlgoritma::tendang_gawang(int& goal_angle, bool& flagGawang, pcData& mPcData, STM32Data& mSTM32Data)
{
    double current_value = goal_angle;

    double kec_rotasi = R2CAlgoritma::calculateOutput(KP_gawang,KI_gawang,KD_gawang,0, goal_angle);

    if(kec_rotasi >= 5)
        {kec_rotasi = 5;}
    else if(kec_rotasi <= -5)
        {kec_rotasi = -5;}

    if ((mSTM32Data.GRIDX >= 9 && mSTM32Data.GRIDX <= 16) && (mSTM32Data.GRIDY >= 12 && mSTM32Data.GRIDY <= 24)){
        if ((mSTM32Data.GRIDX >= 13 && mSTM32Data.GRIDX <= 16))
        {
            //printf("grid pertama\n");
            if((mSTM32Data.KOMPAS >= -45 && mSTM32Data.KOMPAS <= 0) || (mSTM32Data.KOMPAS >= 0 && mSTM32Data.KOMPAS <= 135))
            {
                //printf("k\n");
                mPcData.VZ = Vz_gawang;
                if (flagGawang == 1)
                {
                    mPcData.MOTION = 1;
                    mPcData.HANDLER = 2;
                    mPcData.VZ = 0; 

                    if (goal_angle > angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle < -angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle >= -angle_thresh && goal_angle <= angle_thresh)
                    {
                        //printf("masuk\n");
                        if (mSTM32Data.BOLA == 1){
                            mPcData.MOTION = 1;
                            mPcData.VZ = 0;
                            mPcData.HANDLER = 2;
                            //printf("tendang\n");
                            mPcData.KICK_MODE = 1;
                            mPcData.TENDANG = power_kick;
                            usleep(35000);
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                            return true;
                        }
                        else
                        {
                            //printf("tidak tendang\n");
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                        }
                    }
                }
            }

            if((mSTM32Data.KOMPAS >= -180 && mSTM32Data.KOMPAS <= -45) || (mSTM32Data.KOMPAS >= 135 && mSTM32Data.KOMPAS <= 180))
            {
                mPcData.VZ = -Vz_gawang;
                if (flagGawang == 1)
                {
                    mPcData.MOTION = 1;
                    mPcData.HANDLER = 2;
                    mPcData.VZ = 0; 

                    if (goal_angle > angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle < -angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle >= -angle_thresh && goal_angle <= angle_thresh)
                    {
                        //printf("masuk\n");
                        if (mSTM32Data.BOLA == 1){
                            mPcData.MOTION = 1;
                            mPcData.VZ = 0;
                            mPcData.HANDLER = 2;
                            //printf("tendang\n");
                            mPcData.KICK_MODE = 1;
                            mPcData.TENDANG = power_kick;
                            usleep(35000);
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                            return true;
                        }
                        else
                        {
                            //printf("tidak tendang\n");
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                        }
                    }
                }
            }
        }
        if ((mSTM32Data.GRIDX >= 9 && mSTM32Data.GRIDX <= 12))
        {
            //printf("grid kedua\n");
            if((mSTM32Data.KOMPAS >= -15 && mSTM32Data.KOMPAS <= 0) || (mSTM32Data.KOMPAS >= 0 && mSTM32Data.KOMPAS <= 165))
            {
                //printf("n\n");
                mPcData.VZ = Vz_gawang;
                if (flagGawang == 1)
                {
                    mPcData.MOTION = 1;
                    mPcData.HANDLER = 2;
                    mPcData.VZ = 0; 

                    if (goal_angle > angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle < -angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle >= -angle_thresh && goal_angle <= angle_thresh)
                    {
                        //printf("masuk\n");
                        if (mSTM32Data.BOLA == 1){
                            mPcData.MOTION = 1;
                            mPcData.VZ = 0;
                            mPcData.HANDLER = 2;
                            //printf("tendang\n");
                            mPcData.KICK_MODE = 1;
                            mPcData.TENDANG = power_kick;
                            usleep(35000);
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                            return true;
                        }
                        else
                        {
                            //printf("tidak tendang\n");
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                        }
                    }
                }
            }

            if((mSTM32Data.KOMPAS >= -180 && mSTM32Data.KOMPAS <= -15) || (mSTM32Data.KOMPAS >= 165 && mSTM32Data.KOMPAS <= 180))
            {
                mPcData.VZ = -Vz_gawang;
                if (flagGawang == 1)
                {
                    mPcData.MOTION = 1;
                    mPcData.HANDLER = 2;
                    mPcData.VZ = 0; 

                    if (goal_angle > angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle < -angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle >= -angle_thresh && goal_angle <= angle_thresh)
                    {
                        //printf("masuk\n");
                        if (mSTM32Data.BOLA == 1){
                            mPcData.MOTION = 1;
                            mPcData.VZ = 0;
                            mPcData.HANDLER = 2;
                            //printf("tendang\n");
                            mPcData.KICK_MODE = 1;
                            mPcData.TENDANG = power_kick;
                            usleep(35000);
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                            return true;
                        }
                        else
                        {
                            //printf("tidak tendang\n");
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                        }
                    }
                }
            }
        }
    }
    else if ((mSTM32Data.GRIDX >= 1 && mSTM32Data.GRIDX <= 8) && (mSTM32Data.GRIDY >= 12 && mSTM32Data.GRIDY <= 24)){
        if ((mSTM32Data.GRIDX >= 5 && mSTM32Data.GRIDX <= 8))
        {
            //printf("grid ketiga\n");
            if((mSTM32Data.KOMPAS >= 15 && mSTM32Data.KOMPAS <= 180) || (mSTM32Data.KOMPAS >= -180 && mSTM32Data.KOMPAS <= -165))
            {
                //printf("k\n");
                mPcData.VZ = Vz_gawang;
                if (flagGawang == 1)
                {
                    mPcData.MOTION = 1;
                    mPcData.HANDLER = 2;
                    mPcData.VZ = 0; 

                    if (goal_angle > angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle < -angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle >= -angle_thresh && goal_angle <= angle_thresh)
                    {
                        //printf("masuk\n");
                        if (mSTM32Data.BOLA == 1){
                            mPcData.MOTION = 1;
                            mPcData.VZ = 0;
                            mPcData.HANDLER = 2;
                            //printf("tendang\n");
                            mPcData.KICK_MODE = 1;
                            mPcData.TENDANG = power_kick;
                            usleep(35000);
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                            return true;
                        }
                        else
                        {
                            //printf("tidak tendang\n");
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                        }
                    }
                }
            }

            if((mSTM32Data.KOMPAS >= 0 && mSTM32Data.KOMPAS <= 15) || (mSTM32Data.KOMPAS >= -165 && mSTM32Data.KOMPAS <= 0))
            {
                mPcData.VZ = -Vz_gawang;
                if (flagGawang == 1)
                {
                    mPcData.MOTION = 1;
                    mPcData.HANDLER = 2;
                    mPcData.VZ = 0; 

                    if (goal_angle > angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle < -angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle >= -angle_thresh && goal_angle <= angle_thresh)
                    {
                        //printf("masuk\n");
                        if (mSTM32Data.BOLA == 1){
                            mPcData.MOTION = 1;
                            mPcData.VZ = 0;
                            mPcData.HANDLER = 2;
                            //printf("tendang\n");
                            mPcData.KICK_MODE = 1;
                            mPcData.TENDANG = power_kick;
                            usleep(35000);
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                            return true;
                        }
                        else
                        {
                            //printf("tidak tendang\n");
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                        }
                    }
                }
            }
        }
        if ((mSTM32Data.GRIDX >= 1 && mSTM32Data.GRIDX <= 4))
        {
            //printf("grid keempat\n");
            if((mSTM32Data.KOMPAS >= 45 && mSTM32Data.KOMPAS <= 180) || (mSTM32Data.KOMPAS >= -180 && mSTM32Data.KOMPAS <= -135))
            {
                //printf("n\n");
                mPcData.VZ = Vz_gawang;
                if (flagGawang == 1)
                {
                    mPcData.MOTION = 1;
                    mPcData.HANDLER = 2;
                    mPcData.VZ = 0; 

                    if (goal_angle > angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle < -angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle >= -angle_thresh && goal_angle <= angle_thresh)
                    {
                        //printf("masuk\n");
                        if (mSTM32Data.BOLA == 1){
                            mPcData.MOTION = 1;
                            mPcData.VZ = 0;
                            mPcData.HANDLER = 2;
                            //printf("tendang\n");
                            mPcData.KICK_MODE = 1;
                            mPcData.TENDANG = power_kick;
                            usleep(35000);
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                            return true;
                        }
                        else
                        {
                            //printf("tidak tendang\n");
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                        }
                    }
                }
            }

            if((mSTM32Data.KOMPAS >= 0 && mSTM32Data.KOMPAS <= 45) || (mSTM32Data.KOMPAS >= -135 && mSTM32Data.KOMPAS <= 0))
            {
                mPcData.VZ = -Vz_gawang;
                if (flagGawang == 1)
                {
                    mPcData.MOTION = 1;
                    mPcData.HANDLER = 2;
                    mPcData.VZ = 0; 

                    if (goal_angle > angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle < -angle_thresh)
                    {
                        mPcData.MOTION = 1;
                        mPcData.KECEPATAN = 0;
                        mPcData.HEADING = 0;
                        mPcData.VZ = kec_rotasi*scale;
                        mPcData.HANDLER = 2;
                        return false;
                    }

                    else if (goal_angle >= -angle_thresh && goal_angle <= angle_thresh)
                    {
                        //printf("masuk\n");
                        if (mSTM32Data.BOLA == 1){
                            mPcData.MOTION = 1;
                            mPcData.VZ = 0;
                            mPcData.HANDLER = 2;
                            //printf("tendang\n");
                            mPcData.KICK_MODE = 1;
                            mPcData.TENDANG = power_kick;
                            usleep(35000);
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                            return true;
                        }
                        else
                        {
                            //printf("tidak tendang\n");
                            mPcData.KICK_MODE = 0;
                            mPcData.TENDANG = 0;
                        }
                        
                    }
                }
            }
        }
    }
}

bool R2CAlgoritma::kejar_bola_dekat(int angle_bola,pcData& mPcData, STM32Data& mSTM32Data,int e_distance,int vjauh,int vdeket){
    if(mSTM32Data.BOLA){
        mPcData.MOTION=1;
        mPcData.KECEPATAN=0;
        mPcData.HANDLER=2;
        mPcData.VZ=0;
        PD=0;
        return true;
    }
 
	error_sudut_bola = (0 - angle_bola);

	propotional_sudut = KP * error_sudut_bola;
	derivative_sudut = KD * (error_sudut_bola - last_error_sudut_bola);
	integratif_sudut = KI * (error_sudut_bola + last_error_sudut_bola);

	last_error_sudut_bola = error_sudut_bola;

	PD = propotional_sudut + derivative_sudut; 
    
	if (PD > 10) PD = 9;//5
	else if (PD < -10) PD = -9;

	if (abs(angle_bola) < 7)
	{
		PD = PD / 2;//2;
	}
	else if (abs(angle_bola) > 11)
	{
		if (counter_rem_dekati_bola > 0)
		{
			mPcData.MOTION = 0;
			counter_rem_dekati_bola--;
			return false;
		}
		mPcData.MOTION = 1;
		mPcData.KECEPATAN = 0;
		mPcData.HEADING = 0;
		mPcData.VZ = (PD / 1.8)*10;
		return false;
	}
    
	if (e_distance > 90)
	{
        // printf(">70\n");
		counter_rem_dekati_bola = 25;
		mPcData.MOTION = 1;
		mPcData.KECEPATAN = vjauh;
		mPcData.HEADING = 0;
		mPcData.VZ = (PD/1.8) * 10;//1.4
		mPcData.HANDLER = 2;
	}
    
	if (e_distance <= 90)
	{
        if (counter_rem_dekati_bola > 0)
		{
			mPcData.MOTION = 0;
			counter_rem_dekati_bola--;
			return false;
		}
        // counter_rem_dekati_bola = 10;
		mPcData.MOTION = 1;
		mPcData.KECEPATAN = vdeket;
		mPcData.HEADING = 0;
		mPcData.VZ = (PD / 1.8 * 10); 
		mPcData.HANDLER = 2;
	}
    
	return false;

}

static int angle_bola_hilang = 0;

/////////////////////////////////////////////////////kONFIGURASI STRIKER//////////////////////////////////////////////////

bool R2CAlgoritma::kejar_bola(bool flag_bola, int angle_bola, int V_jauh, int V_dekat, int threshold_jarak, int e_distance, pcData& mPcData, STM32Data& mSTM32Data)
{ 
    //cout << angle_bola << endl;
	error_sudut_bola = (0 - angle_bola);

	propotional_sudut = KP * error_sudut_bola;
	derivative_sudut = KD * (error_sudut_bola - last_error_sudut_bola);
	integratif_sudut = KI * (error_sudut_bola + last_error_sudut_bola);

	last_error_sudut_bola = error_sudut_bola;

	PD = propotional_sudut + derivative_sudut; 
    
	if (PD > 10) PD = 9;//5
	else if (PD < -10) PD = -9;
    
    if(flag_bola == true){
        if (abs(angle_bola) < 5)
        {
            PD = PD / 1;//2;
        }
        else if (abs(angle_bola) > 10)
        {
            if (counter_rem_dekati_bola > 0)
            {
                mPcData.MOTION = 0;
                counter_rem_dekati_bola--;
                return false;
            }
            mPcData.MOTION = 1;
            mPcData.KECEPATAN = 0;
            mPcData.HEADING = angle_bola;
            mPcData.VZ = (PD / 0.7);
            return false;
        }

        if (e_distance < threshold_jarak)
        {
            //printf(">70\n");
            counter_rem_dekati_bola = 15;
            mPcData.MOTION = 1;
            mPcData.KECEPATAN = V_dekat;
            mPcData.HEADING = 0;
            mPcData.VZ = (PD / 0.4);//1.4
            mPcData.HANDLER = 2;
        }

        if (e_distance > threshold_jarak)
        {
            counter_rem_dekati_bola = 15;
            mPcData.MOTION = 1;
            mPcData.KECEPATAN = V_jauh;
            mPcData.HEADING = 0;
            mPcData.VZ = (PD / 0.4); 
            mPcData.HANDLER = 2;
        }
    }
    else
    {
        // angle_bola_hilang = angle_bola;
        mPcData.MOTION = 1;
        mPcData.HEADING = 0;
        mPcData.KECEPATAN = 0;
        mPcData.HANDLER = 0;
    }

    
	return false;
}

int count_oper = 0;

bool R2CAlgoritma::oper_teman(pcData& mPcData, STM32Data& mSTM32Data, int& sudutTemanOmni, int& sudutTemanFront, bool& flagFriendOmni, bool& flagFriendFront)
{
    float kp = 0.3;
    float ki = 0.0;
    float kd = 0.5;
    double setpoint = 0.0;
    int signmove = 0;
    double current_value = sudutTemanFront;
    double kec_rotasi = R2CAlgoritma::calculateOutput(kp,ki,kd,0, sudutTemanFront);

    if(kec_rotasi >= 5)
        {kec_rotasi = 5;}
    else if(kec_rotasi <= -5)
        {kec_rotasi = -5;}


    if (sudutTemanOmni > 0)
    {
        signmove = -1;
    }
    else if (sudutTemanOmni < 0)
    {
        signmove = 1;
    }

    if (signmove == -1)
    {
        //printf("putar kanan\n");
        mPcData.MOTION = 1;
        mPcData.VZ = signmove*10;
        if(flagFriendFront == 1)
        {
            //mPcData.MOTION = 0;
            if (abs(sudutTemanFront) > 3)
            {
                kec_rotasi--;
                mPcData.MOTION = 1;
                mPcData.KECEPATAN = 0;
                mPcData.HEADING = 0;
                mPcData.VZ = kec_rotasi;
                mPcData.HANDLER = 2;
            }
            else
            {
                //mPcData.MOTION = 0;
               // mPcData.VZ = 0;
            
                if (mSTM32Data.BOLA == 1){
                    mPcData.MOTION = 1;
                    mPcData.VZ = 0;
                    mPcData.HANDLER = 2;
                    printf("tendang\n");
                    mPcData.KICK_MODE = 1;
                    mPcData.TENDANG = 5;
                    usleep(35000);
                    mPcData.KICK_MODE = 0;
                    mPcData.TENDANG = 0;
                    return true;
                }
                else
                {
                    printf("tidak tendang\n");
                    mPcData.KICK_MODE = 0;
                    mPcData.TENDANG = 0;
                    return false;
                }
            }
        }
    }
    else if (signmove == 1)
    {
        mPcData.MOTION = 1;
        mPcData.VZ = signmove*10;
        if(flagFriendFront == 1)
        {
            if (abs(sudutTemanFront) > 3)
            {
                kec_rotasi--;
                mPcData.MOTION = 1;
                mPcData.KECEPATAN = 0;
                mPcData.HEADING = 0;
                mPcData.VZ = kec_rotasi;
                mPcData.HANDLER = 2;
            }
            else
            {
                //mPcData.MOTION = 0;
                //mPcData.VZ = 0;

               if (mSTM32Data.BOLA == 1){
                    mPcData.MOTION = 1;
                    mPcData.VZ = 0;
                    mPcData.HANDLER = 2;
                    printf("tendang\n");
                    mPcData.KICK_MODE = 1;
                    mPcData.TENDANG = 5;
                    usleep(35000);
                    mPcData.KICK_MODE = 0;
                    mPcData.TENDANG = 0;
                    return true;
                }
                else
                {
                    printf("tidak tendang\n");
                    mPcData.KICK_MODE = 0;
                    mPcData.TENDANG = 0;
                    return false;
                }
            }
        }
    }
    
}



bool kick;
int count_teman = 0;

bool R2CAlgoritma::lihat_teman_diam(bool param_bola, int angle_teman, pcData& mPcData, STM32Data& mSTM32Data, bool teman1)
{
    float kp = 0.5;
    float ki = 0;
    float kd = 0.2;
    double setpoint = 0.0;
    double current_value = angle_teman;
    double kec_rotasi_lihat_teman = calculateOutput(kp,ki,kd,0, angle_teman);
   
   //cout << "angle = " << angle_teman << endl;
    if (mSTM32Data.BOLA == 1)
    {       
        kec_rotasi_lihat_teman--;
        do{
            if (angle_teman < 1 && angle_teman >-1 )
            {
                cout << "sudah berhadap-hadapan" << endl;
                mPcData.MOTION = 1;
                mPcData.KECEPATAN = 0;
                mPcData.HEADING = 0;
                mPcData.VZ = 0;
            }
            count_teman++;
        }while (count_teman < 10000);
        count_teman = 0;
        kick = true;
        //cout << mPcData.VZ << endl;
        if (angle_teman > 1)
        {
            kick = false;           
            //cout << "teman ada di kanan" << endl;        
            mPcData.MOTION = 1;
            mPcData.KECEPATAN = 0;
            mPcData.HEADING = 0;
            mPcData.VZ = kec_rotasi_lihat_teman;
            mPcData.HANDLER = 1;
            mPcData.KICK_MODE = 0;
            mPcData.TENDANG = 0;
        }
        else if (angle_teman <=-1)
        {
            //printf("%i",mPcData.VZ);
            kick = false;
            //cout << "teman ada di kiri" << endl;
            mPcData.MOTION = 1;
            mPcData.KECEPATAN = 0;
            mPcData.HEADING = 0;
            mPcData.VZ = kec_rotasi_lihat_teman;
            mPcData.HANDLER = 1;
            mPcData.KICK_MODE = 0 ;
            mPcData.TENDANG = 0;
        }
    }
    else if (mSTM32Data.BOLA == 0) {
        kick = false;
        mPcData.MOTION = 0;
        mPcData.KECEPATAN = 0;
        mPcData.HEADING = 0;
        mPcData.VZ =0 ;
        mPcData.HANDLER = 0;
        mPcData.KICK_MODE = 0 ;
        mPcData.TENDANG = 0;
    }

    if(teman1 == false)
    {
        kick = false;
    }

    if (kick == true)
    {   
        mPcData.MOTION = 0;
        mPcData.HANDLER = 0;
        mPcData.KICK_MODE = 1 ;
        mPcData.TENDANG = 6;
        usleep(10000);
		kick = false;
    }
    else
    {
        mPcData.HANDLER = 0;
        mPcData.KICK_MODE = 0;
        mPcData.TENDANG = 0;
    }

    //cout << kick << endl;
}
int pengurang = 0;
bool moving_rotation = false;
bool R2CAlgoritma:: catch_ball(pcData& mPcData, STM32Data& mSTM32Data, Point center, Point ball_intercept, int b_head_deg, bool& Catch, float angle, float e_distance, int catch_angle)
{
    if(Catch == true && moving_rotation == true)
    {
        if(mSTM32Data.BOLA == 0)
        {
            R2CMotion::movingrotation(angle, mPcData, mSTM32Data);
        }
        else if(mSTM32Data.BOLA == 1)
        {
            Catch = false;
            moving_rotation = false;
            R2CMotion::Motion(mPcData, 0, 0, 0, 0, 0);
        }
    }

    if(Catch == true && moving_rotation == false)
    {
        if(mSTM32Data.BOLA == 1)
        {
            Catch = false;
            R2CMotion::Motion(mPcData,1, 0, 0, 0, 1);
        }
        else if(e_distance <= 55)
        {
            moving_rotation == true;
        }
        else if((catch_angle < 4) && (catch_angle > -4))
        {
            R2CMotion::Motion(mPcData, 1, 0, 0, 0, 1);
        }
        else if(norm(center - ball_intercept) < 115)
        {
            if(b_head_deg < 0 && catch_angle >= 4 )
            {
                mPcData.HEADING = 90;
                mPcData.KECEPATAN = 100;
                int kp = R2CAlgoritma::calculateOutput(0.1,0,0,90,mSTM32Data.KOMPAS*-1); //1.4
                mPcData.VZ = kp;
            }
            else
            {
                mPcData.HEADING = -90;
                mPcData.KECEPATAN = 100;
                int kp = R2CAlgoritma::calculateOutput(0.1,0,0,90,mSTM32Data.KOMPAS*-1); //1.4
                mPcData.VZ = kp;
            }
        }
        else
        {
            R2CMotion::Motion(mPcData, 1, 0, 0, 0, 1);
        }  
        printf("pengurang: %i",pengurang);
    }
    return false;
}
/*
Tes coba
void R2CAlgoritma::calculate_ball_speed(float ball_angle, float e_distance, pcData& mPcData, STM32Data& mSTM32Data)
{
    float delta_angle = 0.0;
    float delta_angle_rad = 0.0;
    float Rball = 0.0; //jarak sebenarnya
    float X_to_robot = 0.0;
    float Y_to_robot = 0.0;

    float X_ball = 0.0;
    float Y_ball = 0.0;
    
    
    Rball = -1804 + (62.8*e_distance) - (0.7*e_distance*e_distance) + ((2.68*e_distance*e_distance*e_distance)/(10*10*10));

    delta_angle = ball_angle - mSTM32Data.KOMPAS;
    delta_angle_rad = delta_angle*M_PI/180;

    X_to_robot = Rball * cos(delta_angle_rad);
    Y_to_robot = Rball * sin(delta_angle_rad);

    X_ball = X_to_robot + mSTM32Data.GRIDX;
    Y_ball = Y_to_robot + mSTM32Data.GRIDY;

    printf("%.2f|%.2f\n", delta_angle, delta_angle_rad);
}
*/

/*
bool R2CAlgoritma::lihat_teman_diam(bool param_bola, int angle_teman, pcData& mPcData, STM32Data& mSTM32Data, bool teman1)
{
    float kp = 0.5;
    float ki = 0;
    float kd = 0.2;
    double setpoint = 0.0;
    double current_value = angle_teman;
    double kec_rotasi_lihat_teman = calculateOutput(kp,ki,kd,0, angle_teman);
    

    // if (angle_teman >= 10)
    // {
    //     kec_rotasi_lihat_teman = -15;
    // }
    // else if (angle_teman <= -10)
    // {
    //     kec_rotasi_lihat_teman = 15;
    // }
    // else if (angle_teman <=5 && angle_teman >= 0)
    // {
    //     kec_rotasi_lihat_teman = -7;
    // }
    // else if (angle_teman >=-5 && angle_teman <= 0)
    // {
    //     kec_rotasi_lihat_teman = 7;
    // }

    if (kec_rotasi_lihat_teman >= 10)
    {
        kec_rotasi_lihat_teman = 20;
    }
    else if (kec_rotasi_lihat_teman <= -10)
    {
        kec_rotasi_lihat_teman = -20;
    }
    else if (kec_rotasi_lihat_teman <=5 && kec_rotasi_lihat_teman >= 0)
    {
        kec_rotasi_lihat_teman = 7;
    }
    else if (kec_rotasi_lihat_teman >=-5 && kec_rotasi_lihat_teman <= 0)
    {
        kec_rotasi_lihat_teman = -7;
    }

    
   cout << "angle = " << angle_teman << endl;
    if (mSTM32Data.BOLA == 0)
    {
        kec_rotasi_lihat_teman--;
        if (angle_teman > 1)
        {
            mPcData.MOTION = 1;
            mPcData.KECEPATAN = 0;
            mPcData.HEADING = 0;
            mPcData.VZ = kec_rotasi_lihat_teman;
            mPcData.HANDLER = 1;
            mPcData.KICK_MODE = 0;
            mPcData.TENDANG = 0;
            return false;
        }
        else if (angle_teman < -1)
        {
            mPcData.MOTION = 1;
            mPcData.KECEPATAN = 0;
            mPcData.HEADING = 0;
            mPcData.VZ = kec_rotasi_lihat_teman;
            mPcData.HANDLER = 1;
            mPcData.KICK_MODE = 0;
            mPcData.TENDANG = 0;
            return false;
        }
        else if (angle_teman >= -1 && angle_teman <= 1 )
        {
            //printf("tendang");
            mPcData.VZ = 0;
            //return true;
            if(stop == 0)
                {
                    time_setpoint++;
                    printf("%i\n", time_setpoint);
                    if (time_setpoint == 100)
                    {
                        stop = 1;
                        mPcData.KICK_MODE = 1;
                        mPcData.TENDANG = 1;
                        printf("tendang");    
                    }
                }
            mPcData.MOTION = 0;
            mPcData.KICK_MODE = 0;
            mPcData.TENDANG = 0;
            
        }
    }

    // if (mSTM32Data.BOLA == 0)
    // {       
    //     kec_rotasi_lihat_teman--;
    //     do{
    //         if (angle_teman < 3 && angle_teman >-3 )
    //         {
    //             cout << "sudah berhadap-hadapan" << endl;
    //             mPcData.MOTION = 1;
    //             mPcData.KECEPATAN = 0;
    //             mPcData.HEADING = 0;
    //             mPcData.VZ = 0;
    //         }
    //         count_teman++;
    //     }while (count_teman < 10000);
    //     count_teman = 0;
    //     kick = true;
    //     //cout << mPcData.VZ << endl;
    //     if (angle_teman > 3)
    //     {
    //         kick = false;           
    //         //cout << "teman ada di kanan" << endl;        
    //         mPcData.MOTION = 1;
    //         mPcData.KECEPATAN = 0;
    //         mPcData.HEADING = 0;
    //         mPcData.VZ = kec_rotasi_lihat_teman;
    //         mPcData.HANDLER = 1;
    //         mPcData.KICK_MODE = 0;
    //         mPcData.TENDANG = 0;
    //     }
    //     else if (angle_teman <=-3)
    //     {
    //         //printf("%i",mPcData.VZ);
    //         kick = false;
    //         //cout << "teman ada di kiri" << endl;
    //         mPcData.MOTION = 1;
    //         mPcData.KECEPATAN = 0;
    //         mPcData.HEADING = 0;
    //         mPcData.VZ = kec_rotasi_lihat_teman;
    //         mPcData.HANDLER = 1;
    //         mPcData.KICK_MODE = 0 ;
    //         mPcData.TENDANG = 0;
    //     }
    // }
    // else if (mSTM32Data.BOLA == 1) {
    //     kick = false;
    //     mPcData.MOTION = 0;
    //     mPcData.KECEPATAN = 0;
    //     mPcData.HEADING = 0;
    //     mPcData.VZ =0 ;
    //     mPcData.HANDLER = 0;
    //     mPcData.KICK_MODE = 0 ;
    //     mPcData.TENDANG = 0;
    // }

    // if(teman1 == false)
    // {
    //     kick = false;
    // }

    // if (kick == true)
    // {   
    //     mPcData.MOTION = 0;
    //     mPcData.HANDLER = 0;
    //     mPcData.KICK_MODE = 1 ;
    //     mPcData.TENDANG = 6;
    //     usleep(10000);
	// 	kick = false;
    // }
    // else
    // {
    //     mPcData.HANDLER = 0;
    //     mPcData.KICK_MODE = 0;
    //     mPcData.TENDANG = 0;
    // }

    //cout << kick << endl;
}
*/