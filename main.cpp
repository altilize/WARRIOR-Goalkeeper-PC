#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "library/UDPListen.h"
#include "library/UDPSend.h"
#include "library/TypeData.h"
#include "library/Camera.h"
#include "library/Citra.h"
#include "library/InisiasiSerial.h"
#include "library/KomunikasiMikro.h"
#include "library/Algoritma.h"
#include "library/BackCamera.h"
#include "library/InisiasiSerial.h"
#include "library/Motion.h"

#define CITRA
#define KOMUNIKASI

using namespace std;
using namespace cv;

Serial USBSTM322;
STM32Data mSTM32Data;
pcData mPcData;
BasestationData basestationData;
unsigned char dataMasuk[100];
VideoCapture cap1, cap2;

Mat frame_omni, frame_depan;
Mat frame_lapangan, frame_bola, frame_gawangDepan, frame_gawangOmni;
Mat frame_musuh, frame_musuhDepan;
Mat frame_teman, frame_temanDepan;
float ball_angle, e_distance;
int coordinate_x, coordinate_y;
int signangle;

int goalAngle, goalWidth;
float goalDistance;
int angle_goalPost_left;
int angle_golaPost_right;

bool param_bola = false;
bool param_gawang = false;
bool param_tiang = false;
bool param_teman1 = false, param_teman2 = false;
unsigned char dataUDP[20];
int count_kejar = 0;

Mat frame_shifted_omni, frame_masked_omni, frame_rotation_omni;

Point2f koordinat_teman1;
Point2f koordinat_teman2;
int angle_temen1, angle_temen2;
int jarak_temen1, jarak_temen2;

Point2f koordinat_musuh1;
Point2f koordinat_musuh2;
Point2f koordinat_musuh3;
Point centerEnemy;
float angle_musuh1, angle_musuh2, angle_musuh3;
bool state_lihat = false;
bool lihat_bola = false;
bool pindahgrid1 = false;
bool tendang_teman1 = false;
float angle;
int angleFrontFriend1, angleFrontFriend2;
bool flagFrontFriend1 = false;
bool flagFrontFriend2 = false;
bool flagOmniFriend = false;

int angleFrontEnemy1, angleFrontEnemy2;
bool flagFrontEnemy1 = false;
bool flagFrontEnemy2 = false;
bool flagOmniEnemy = false;

int nLines_obstacle =64;
vector<int>jarak_obstacle(nLines_obstacle, 0);
vector<float>angle_obstacle(nLines_obstacle, 0);

Point b_predict_pos;
Point r_ball_position;
Point ball_intercept;
int catch_heading;
float b_head;
int b_head_deg;

bool Catch = false;
/////////////////////// Variable Lomba 2024////////////////////////////////
bool Koff = false;
bool Koffenemy = false;
bool Koffstop = false;
bool Koffputar = false;
bool hindar=false;
bool statusconnect=true;
bool detectObs = false;
bool hindarMusuh = false;
bool kbolaHindar = false;
bool skill1=false;

bool kejar = false;
bool pindahgrid = false;
bool tendang = false;
bool oper_teman = false;
bool kejar_cepat = false;
bool geser = false;
bool deadstop=false;

bool pointChosen = false;
bool pointplay = true;
bool done = false;
bool pla = false;
int konfirm = 0;
bool hadap = true;
bool reset_mode = true;
int gridx_setpoint = 5;
int gridy_setpoint = 15;
bool oper = false;
bool not_start = false;
bool giliran = true;
bool flagbola_current = true;
bool flagbola_previous = true;
bool tes_hadap = true;
bool tes_gawang = true;
int jumlah_operan =0;
bool bola_oper = false;
bool siap_nendang = false;
bool tendangbosq = false;

float avg_angle = 0, sum_angle = 0, num_angle = 0;
float kp;
int konfirm2 = 0;
///////////////
bool mode_1 = false;
bool mode_2 = false;
bool mode_3 = false;
///////////////////////////////////////////////////////////////////////////
int posisixobjek = 4;
int posisiyobjek =  18;
int posisixobjek2 = 0;
int posisiyobjek2 = 0;
bool adaobjek = false;
bool majulurus = false;
bool stateobjek = false;
bool initval = false;
bool rotation = false;
bool muter = false;
bool tendang_gawang = false;

///////////////////////////////testing new pindah grid//////////////////

bool tespindahgrid = true;

//////////////////////////////////////////////////////////////////
void mode_1_play();
void mode_2_play();
void* thread_citra(void* arg) {
    namedWindow("Kamera Depan", WINDOW_AUTOSIZE);
    namedWindow("Kamera Omni", WINDOW_AUTOSIZE);
    

    while (true) {
        Point center (frame_rotation_omni.cols/2, frame_rotation_omni.rows/2);
        r_ball_position = Point(coordinate_x, coordinate_y);
        cap1 >> frame_omni;
        cap2 >> frame_depan;

        frame_shifted_omni = R2CCitra::shifting_frame(frame_omni, ParameterFrame.geser_frame_x, ParameterFrame.geser_frame_y);
        frame_masked_omni = R2CCitra::masking_frame(frame_shifted_omni, ParameterFrame.radius_mask);

        frame_rotation_omni = R2CCitra::rotation_frame(frame_masked_omni, ParameterFrame.sudut_putaran_omni);
  
        flip(frame_rotation_omni, frame_rotation_omni, 1);
        R2CCitra::segmentasi_warna_omni(frame_rotation_omni, frame_bola, frame_lapangan, frame_musuh, frame_teman);
        R2CCitra::segmentasi_warna_depan(frame_depan, frame_gawangDepan, frame_temanDepan, frame_musuhDepan);
        R2CCitra::convex_hull(frame_rotation_omni, frame_lapangan);
        R2CCitra::deteksi_bola_omni(param_bola, frame_rotation_omni, frame_bola, angle, e_distance, coordinate_x, coordinate_y);
        R2CCitra::deteksi_teman_depan(frame_depan, frame_temanDepan, angleFrontFriend1, angleFrontFriend2, flagFrontFriend1, flagFrontFriend2);
        R2CCitra::deteksi_teman_omni(frame_rotation_omni, frame_teman, koordinat_teman1, angle_temen1, jarak_temen1, flagOmniFriend);
        R2CCitra::deteksi_musuh_depan(frame_depan, frame_musuhDepan, angleFrontEnemy1, angleFrontEnemy2, flagFrontEnemy1, flagFrontEnemy2, centerEnemy);
        R2CCitra::ball_predict(frame_rotation_omni, mPcData, mSTM32Data, r_ball_position, center, catch_heading, ball_intercept, b_predict_pos);
        b_head = atan2(r_ball_position.y - b_predict_pos.y, r_ball_position.x - b_predict_pos.x);
        b_head_deg = (int)((b_head * 180 / CV_PI) + 270) % 360 - 180;
        R2CCitra::deteksi_objek_hitam(frame_depan, nLines_obstacle, jarak_obstacle, angle_obstacle, avg_angle, sum_angle, num_angle, mSTM32Data);
        //circle(frame_depan, Point((int)avg_angle, 150), 20, Scalar(0,255,255), -1);
        R2CAlgoritma::catch_ball(mPcData, mSTM32Data, center, ball_intercept, b_head_deg, Catch, angle, e_distance, catch_heading);
        mPcData.MOTION = 1;
        mPcData.HANDLER = 2;
        R2CCitra::deteksi_gawang_depan(frame_depan, frame_gawangDepan, param_gawang, goalAngle, angleFrontEnemy1, centerEnemy, mSTM32Data, avg_angle);

        // printf("grid x: %i grid y: %i dataudp1: %i dataudp2: %i datadup3: %i dataudp: %i\n",posisixobjek,posisiyobjek,dataUDP[0],dataUDP[1],dataUDP[2],dataUDP[3]);
        // printf("%i state: %i \n",mSTM32Data.BUTTON,rotation);
        // R2CMotion::sampingkiper(mPcData,mSTM32Data);
        /////////////////////////////// MODE 1 ///////////////////////////////////////////////
        mPcData.HEADING = 0;
        mPcData.KECEPATAN = 100;
        printf("bola: %i siaptendang: %i tendangbosq: %i tespindahgrid: %i skill: %i \n",mSTM32Data.BOLA,siap_nendang,tendangbosq,tespindahgrid,basestationData.skillRobot);
        if ((basestationData.skillRobot == 3 || mSTM32Data.BUTTON == 1) && reset_mode == true)
        {
            printf("jalan mas leooooooo");
            if (!pla)
            {
                pla = true;
                reset_mode = false;
                // mode_1 = true;
                tespindahgrid = false;
            }
            else{
                pla = pla;
            }
        }
    if(mSTM32Data.BUTTON == 6){
        Catch = true;
    }
        if(basestationData.skillRobot == 11 && tespindahgrid == false){
            tendangbosq = true;
        }
        if(!tespindahgrid){
            tespindahgrid = R2CMotion::pindahgrid2(5,11,mSTM32Data,mPcData,100,70,90);//roda3
            // siap_nendang= R2CMotion::hadap_teman(mPcData,mSTM32Data,angleFrontFriend1,angle,flagFrontFriend1,mSTM32Data.BOLA,tendangbosq);
            // if((siap_nendang) && (tendangbosq) && (!mSTM32Data.BOLA)){
            //     tespindahgrid = true;
            //     siap_nendang = false;
            //     tendangbosq = false;
            //     reset_mode = true;
            //     pla = false;
            // }
        }
        if(tespindahgrid== true && pla == true){
            pla = !R2CMotion::pindahgrid2(7,16,mSTM32Data,mPcData,100,70,-90);
        }
        if(pla == false && reset_mode ==false){
            reset_mode = R2CMotion::pindahgrid2(1,16,mSTM32Data,mPcData,100,70,90);
        }
    if (mSTM32Data.BUTTON==3){
            mPcData.KICK_MODE=1;
            mPcData.TENDANG=4;
            usleep(60000);
            mPcData.KICK_MODE=0;
            mPcData.TENDANG=0;
            printf("berhasil mas toto\n");
            // R2CMotion::setgrid(mPcData,1,6,19,90);
        }
        if(mode_1){
            mode_1_play();
        }

    //     if(mSTM32Data.BUTTON == 8){
    //         rotation = true;
    //         printf("true");   
    //     }
        if(mSTM32Data.BUTTON == 19)
        {
            tendang_gawang = true;
        }

        if(tendang_gawang == true)
        {
            if((goalAngle <= 2 && goalAngle >= -2))
            {
                kp = 0;
                konfirm2++;
            }
            else
            {
                kp = R2CAlgoritma::calculateOutput(1,0,0.425,0,goalAngle);
                konfirm2 = 0;
            }

            if(konfirm2 > 10)
            {
                konfirm2 = 0;
                mPcData.KICK_MODE = 1;
                mPcData.TENDANG = 15;
                usleep(60000);
                mPcData.KICK_MODE = 0;
                mPcData.TENDANG = 0;
                tendang_gawang = false;
            }
            mPcData.VZ = kp;
        }
        
        if(mSTM32Data.BUTTON == 5){
            tes_hadap = false;
        }
        if(!tes_hadap){
            tes_hadap = R2CMotion::hadap_teman(mPcData,mSTM32Data,angleFrontFriend1,angle,flagFrontFriend1,mSTM32Data.BOLA,tendangbosq);
        }

        if(mSTM32Data.BUTTON == 2){
            // R2CMotion::setgrid(mPcData,1,16,16,90);//roda 4
            R2CMotion::setgrid(mPcData,1,1,16,-90);
            // R2CMotion::setgrid(mPcData,0,16,8,0);
            printf("grid set\n");
        }
        else{
            R2CMotion::setgrid(mPcData,0,0,0,0);
        }
        if(mSTM32Data.BUTTON == 1){
            pointplay = false;
            printf("start\n");
        }
        if(rotation){
            muter = R2CMotion::mengitariobjek(angle,180,mPcData,mSTM32Data);
            if(muter){
                rotation = false;
            }
        }
        if(muter){
            R2CMotion::movingrotation(angle,mPcData,mSTM32Data);
        }
        //imshow("Kamera Omni", frame_masked_omni);
        imshow("Kamera Depan", frame_depan);
        imshow("Kamera Omni", frame_rotation_omni);
        
        num_angle = 0;
        avg_angle = 0;
        sum_angle = 0;
        if (waitKey(5) == 27) {
            break;
        }
    }
    pthread_exit(NULL);
}

void* thread_komunikasi(void* arg) {
    while (1) {
        R2CKomunikasiSTM32::STM32init(USBSTM322);
        R2CKomunikasiSTM32::sendData(USBSTM322, mPcData);
        R2CKomunikasiSTM32::parseSTM32Data(mSTM32Data, dataMasuk);
        USBSTM322.readserial(dataMasuk);
    }
    pthread_exit(NULL);
}

void* receiveBasestation(void *add) {
    UDPListen::openUDP();
    while(true){
        UDPListen::receiveBasestation(dataUDP);
        if(dataUDP[0]==1){
            if(dataUDP[1] == 1){
                posisixobjek = 4;//dataUDP[2];
                posisiyobjek = 18;//dataUDP[3];
            }
        if (dataUDP[1] == 252){
            basestationData.manualRobot = dataUDP[2];
                if (basestationData.manualRobot == 119){
                    printf("maju\n");
                    deadstop=false;
                    R2CMotion::Motion(mPcData,1,0,15,0,2);
                }

                else if (basestationData.manualRobot == 100){
                    printf("kiri\n");
                    deadstop=false;
                    R2CMotion::Motion(mPcData,1,90,15,0,2);
                }

                else if (basestationData.manualRobot == 97){
                    printf("kanan\n");
                    deadstop=false;
                    R2CMotion::Motion(mPcData,1,-90,15,0,2);
                }

                else if (basestationData.manualRobot == 115){
                    printf("mundur\n");
                    deadstop=false;
                    R2CMotion::Motion(mPcData,1,180,15,0,2);
                }

                else if (basestationData.manualRobot == 113){
                    printf("kiri\n");
                    deadstop=false;
                    R2CMotion::Motion(mPcData,1,0,0,15,2);
                }

                else if (basestationData.manualRobot == 101){
                    printf("kanan\n");
                    deadstop=false;
                    R2CMotion::Motion(mPcData,1,0,0,-15,2);
                }
                else if (basestationData.manualRobot == 90){
                    printf("stop\n");
                    deadstop=false;
                    R2CMotion::Motion(mPcData,0,0,0,0,0);
                }
                        
                else if(basestationData.manualRobot == 123){

                    printf("stopmode\n");
                    Koff=false;
                    skill1=false;
                    tendang=false;
                    oper_teman=false;
                    lihat_bola=false;
                    geser=false;
                    kejar_cepat=false;
                    deadstop=true;
                            
                }
                
    }
            else if (dataUDP[1]==251){
                
            }
            else if (dataUDP[1] == 254){
                // printf("masuk254\n");
                basestationData.skillRobot = dataUDP[2];
                if (basestationData.skillRobot==1){
                    printf("skill1 - kejar bola\n");
                    deadstop=false;
                    tendang=false;
                    oper_teman=false;
                    lihat_bola=false;
                    geser=false;
                    kejar_cepat=false;
                    Koff=false;
                    skill1=true;

                }
                if (basestationData.skillRobot==2)
                {
                    printf("skill2 - tendang gawang\n");
                    deadstop=false;
                    skill1=false;
                    oper_teman = false;
                    lihat_bola=false;
                    geser=false;
                    kejar_cepat=false;
                    Koff=false;
                    tendang=true;
                }
                if(basestationData.skillRobot == 3)
                {
                    printf("skill3 - oper teman\n");
                    deadstop=false;
                    skill1=false;
                    tendang = false;
                    lihat_bola=false;
                    geser=false;
                    kejar_cepat=false;
                    Koff=false;
                    oper_teman=true;
                }
                if(basestationData.skillRobot == 4)
                {
                    printf("skill4 - lihat bola diam\n");
                    deadstop=false;
                    skill1=false;
                    tendang = false;
                    oper_teman=false;
                    lihat_bola=true;
                    geser=false;
                    kejar_cepat=false;
                    Koff=false;
                    lihat_bola=true;
                }
                if(basestationData.skillRobot == 5)
                {
                    printf("skill5 - lihat bola geser\n");
                    deadstop=false;
                    skill1=false;
                    tendang = false;
                    oper_teman=false;
                    lihat_bola=false;
                    kejar_cepat=false;
                    Koff=false;
                    geser=true;
                }
                if(basestationData.skillRobot == 6)
                {
                    printf("skill6 - kejar bola cepat\n");
                    deadstop=false;
                    skill1=false;
                    tendang = false;
                    oper_teman=false;
                    lihat_bola=false;
                    geser=false;
                    Koff=false;
                    kejar_cepat=true;
                }

                if(basestationData.skillRobot == 7){
                    printf("skill7 - tendang imut\n");
                    deadstop=false;
                    mPcData.KICK_MODE=1;
                    mPcData.TENDANG=3;
                    usleep(60000);
                    mPcData.KICK_MODE=0;
                    mPcData.TENDANG=0;
                }

                if(basestationData.skillRobot == 10){
                    printf("skill7 - tendang mantap\n");
                    deadstop=false;
                    mPcData.KICK_MODE=1;
                    mPcData.TENDANG=8;
                    usleep(40000);
                    mPcData.KICK_MODE=0;
                    mPcData.TENDANG=0;
                }
                 
            }
            else if (dataUDP[1]==255){
                basestationData.kompasFlag = dataUDP[2];
                basestationData.valueCompas = dataUDP[3];
                if (basestationData.kompasFlag == 1){
                    basestationData.valueCompas *=-1;
                }
                basestationData.gridX =  dataUDP[4];
                basestationData.gridY = dataUDP[5];
                printf("%i|%i", basestationData.gridX, basestationData.gridY);
                deadstop=false;
                skill1=false;
                oper_teman = false;
                tendang=false;
                kejar_cepat=false;
                geser=false;
                lihat_bola=false;
                Koff=true;
                Koffputar=false;
            }
            else if(dataUDP[1] == 1)
            {
                if(dataUDP[2] == 1)
                {
                    printf("standby\n");
                }
            }
        }
    }
    
    return NULL;
}

void* sendBasestation(void *add) {

    while (1)
    {
    unsigned char senddata[17];
        
    senddata[0]=1;

    if (mSTM32Data.KOMPAS<0){
        senddata[1]=1;
        senddata[2]=mSTM32Data.KOMPAS *-1;
    }

    else{
        senddata[1]=0;
        senddata[2]=mSTM32Data.KOMPAS;
    }
    
    int Xpos = (int)mSTM32Data.XPOS;
    senddata[3]=(Xpos >> 8);
    senddata[4]=Xpos;

    int Ypos = (int)mSTM32Data.YPOS;
    senddata[5]=(Ypos >>8);
    senddata[6]=Ypos;

    int real_angle = (int)angle + mSTM32Data.KOMPAS;

    if (real_angle<0){
        signangle=1;
    }
    else{
        signangle=0;
    }

    senddata[7]= signangle;

    int sendangle=(int)real_angle;
    if (signangle){
        sendangle *= -1;
    }
    else {
        sendangle=sendangle;
    }
    senddata[8]=sendangle;
    senddata[9]=siap_nendang;
    senddata[10]=0;
    senddata[11]=0;
    senddata[12]=0;
    senddata[13]=0;
    senddata[14]=basestationData.skillRobot;
    senddata[15]=mSTM32Data.BOLA;
    senddata[16]=(int)statusconnect;
    usleep(50000);
    UDPSend::sendBasestation(senddata);
    } 
    return NULL;
}

int main() {

    R2CCitra::init();
    
    Camera cameraAtas = R2CCamera::loadCameraByPath(KAMERA_omni);
    Camera cameraDepan = R2CCamera::loadCameraByPath(KAMERA_depan);

    R2CCamera::setParameterCamera(cameraAtas);
    R2CCamera::setParameterCamera(cameraDepan);

    cap1.open(cameraAtas.index, cv::CAP_V4L2);
    cap1.set(CAP_PROP_FRAME_WIDTH, 640);
    cap1.set(CAP_PROP_FRAME_HEIGHT, 480);
    //cap1.set(CAP_PROP_AUTOFOCUS, 0);
    cap1.set(CAP_PROP_AUTO_EXPOSURE, 0);

    R2CKomunikasiSTM32::STM32init(USBSTM322);
    cap2.open(cameraDepan.index, cv::CAP_V4L2);
    cap2.set(CAP_PROP_FRAME_WIDTH, 640);
    cap2.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap2.set(CAP_PROP_AUTOFOCUS, 0);
    //cap2.set(CAP_PROP_AUTO_EXPOSURE, 1);

    if (!cap1.isOpened()) {
        cout << "ERROR! Unable to open camera omni!\n" << endl;
        return -1;
    }

    if (!cap2.isOpened()) {
        cout << "ERROR! Unable to open camera depan!\n" << endl;
        return -1;
    }
    pthread_t citra_thread;
    if (pthread_create(&citra_thread, NULL, thread_citra, NULL) != 0) {
        cerr << "Error creating citra_thread" << endl;
        return 1;
    }

    pthread_t komunikasi_thread;
    if (pthread_create(&komunikasi_thread, NULL, thread_komunikasi, NULL) != 0) {
        cerr << "Error creating komunikasi_thread" << endl;
        return 1;
    }

    pthread_t receive_thread;
    if (pthread_create(&receive_thread, NULL, receiveBasestation, NULL) != 0) {
        cerr << "Error creating receive_thread" << endl;
        return 1;
    }

    pthread_t send_thread;
    if (pthread_create(&send_thread, NULL, sendBasestation, NULL) != 0) {
        cerr << "Error creating send_thread" << endl;
        return 1;
    }

    pthread_join(citra_thread, NULL);
    pthread_join(send_thread, NULL);
    pthread_join(komunikasi_thread, NULL);
    pthread_join(receive_thread, NULL);
    return 0;
}

void mode_1_play(){
    if(pla){
        // pointplay = R2CMotion::pindahgrid(12,12,mSTM32Data,mPcData,12,9); //roda4
        pointplay = R2CMotion::pindahgrid2(5,11,mSTM32Data,mPcData,200,100,90);//roda3
        if(pointplay){
            done = true;
            pla = false;
            }
        }
    if(done){
            if(mSTM32Data.BOLA == 0){
                R2CMotion::movingrotation(angle,mPcData,mSTM32Data);
            }
            else{
                done = false;
                hadap = false;
                not_start = true;
                R2CMotion::Motion(mPcData,0,0,0,0,2);
            }
        }
    if (mSTM32Data.BUTTON==3){
            mPcData.KICK_MODE=1;
            mPcData.TENDANG=11;
            usleep(60000);
            mPcData.KICK_MODE=0;
            mPcData.TENDANG=0;
            printf("berhasil mas toto\n");
        }
    if(mSTM32Data.BUTTON == 20){
            tes_gawang = false;
        }
        // printf("flag sekarang: %i | flag sebelumnya: %i | skill:%i | operan: %i",flagbola_current,flagbola_previous,basestationData.skillRobot,jumlah_operan);
        // printf("Catch: %i Bola Oper: %i hadap: %i  button: %i tendangndak: %i skill: %i | adaobjek %i \n",Catch,bola_oper,hadap,mSTM32Data.BUTTON,tendangbosq,basestationData.skillRobot,adaobjek);
        // printf("grid x: %i grid y: %i\n",posisixobjek,posisiyobjek);
    if(basestationData.skillRobot == 11){
       tendangbosq = true;
    }
    else{
       tendangbosq = false;
    }
    if(!giliran){
        flagbola_current = mSTM32Data.BOLA;
    }
    if(majulurus){
        if(!stateobjek){
            if(!initval){
                if(gridy_setpoint+3 < 21){
                    gridy_setpoint +=3;
                }
                else{
                    gridy_setpoint -=3;
                }
                initval =true;
            }
            stateobjek = R2CMotion::pindahgrid2(gridx_setpoint,gridy_setpoint,mSTM32Data,mPcData,130,90,90);
        }
        else{
            if(initval){
                gridx_setpoint +=3;
                initval = false;
            }
            stateobjek = !R2CMotion::pindahgrid2(gridx_setpoint,gridy_setpoint,mSTM32Data,mPcData,130,90,90);
            if(!stateobjek){
                adaobjek = false;
                majulurus = false;
            }
        }
    }
if(!majulurus){
    if(jumlah_operan<=5){
        if((!hadap) && flagbola_current == flagbola_previous){
            hadap = R2CMotion::hadap_teman(mPcData,mSTM32Data,angleFrontFriend1,angle,flagFrontFriend1,mSTM32Data.BOLA,tendangbosq);
            if(hadap){
                giliran = false;
            }
            // printf("jadi dong!!!");
        }
        else if ((flagbola_current != flagbola_previous) && (not_start) && (!Catch) && (!bola_oper)){
            
                oper = R2CMotion::pindahgrid2(gridx_setpoint,gridy_setpoint,mSTM32Data,mPcData,110,70,90);
                if((oper) && (adaobjek)){
                    majulurus = true;
                }
            if((oper) && (!majulurus)){
                if((gridy_setpoint+2)<21){
                    if((gridy_setpoint+2) == posisiyobjek){
                        gridy_setpoint +=1;
                        gridx_setpoint -=3;
                        adaobjek = true;
                    }
                    else if((gridy_setpoint+1) == posisiyobjek){
                        gridy_setpoint +=0;
                        gridx_setpoint -=3;
                        adaobjek = true;
                    }
                    else{
                        gridy_setpoint+=2;    
                    }
                }
                else{
                    if((gridy_setpoint-2) == posisiyobjek){
                        gridy_setpoint -=1;
                        gridx_setpoint -=3;
                        adaobjek = true;
                    }
                    else if((gridy_setpoint-1) == posisiyobjek){
                        gridy_setpoint +=0;
                        gridx_setpoint -=3;
                        adaobjek = true;
                    }
                    else{
                        gridy_setpoint-=2;    
                    }
                }
                jumlah_operan++;
                oper = false;
                hadap = false;
                flagbola_previous = flagbola_current;
                bola_oper = true;                
            }
        }   
    }
    else{
        tes_gawang = false;
    }
}
    if(mSTM32Data.BUTTON == 5){
        tes_hadap = false;
    }

    if(!tes_hadap){
        tes_hadap = R2CMotion::hadap_teman(mPcData,mSTM32Data,angleFrontFriend1,angle,flagFrontFriend1,mSTM32Data.BOLA,tendangbosq);
    }
    if(!tes_gawang){
        // tes_gawang = R2CAlgoritma::tendang_gawang(goalAngle, param_gawang, mPcData, mSTM32Data);
        tes_gawang = R2CMotion::putar_gawang(mPcData,mSTM32Data);
        if (tes_gawang){
            mPcData.VZ = 0;
            tendang_gawang = true;
            // reset_mode = true;
        }
            

    }

    if(tendang_gawang == true)
        {
            if((goalAngle <= 2 && goalAngle >= -2))
            {
                kp = 0;
                konfirm2++;
            }
            else
            {
                kp = R2CAlgoritma::calculateOutput(1,0,0.425,0,goalAngle);
                konfirm2 = 0;
            }

            if(konfirm2 > 10)
            {
                konfirm2 = 0;
                mPcData.KICK_MODE = 1;
                mPcData.TENDANG = 13;
                usleep(60000);
                mPcData.KICK_MODE = 0;
                mPcData.TENDANG = 0;
                tendang_gawang = false;
                if (!tendang_gawang){
                    mPcData.VZ = 0;
                    mode_1 = false;
                    reset_mode = true;
                }
            }
            mPcData.VZ = kp;
        }

    if((hadap) && (!mSTM32Data.BOLA)&&(not_start) && (bola_oper)){
        Catch = true;
    }
    else if((hadap) && (mSTM32Data.BOLA)&&(not_start) && (bola_oper)){
        Catch = false;
        bola_oper = false;
    }
}

void mode_2_play(){
    if(pla){
        // pointplay = R2CMotion::pindahgrid(12,12,mSTM32Data,mPcData,12,9); //roda4
        pointplay = R2CMotion::pindahgrid(12,11,mSTM32Data,mPcData,200,100);//roda3
        if(pointplay){
            done = true;
            pla = false;
            }
        }
    if(done){
            if(mSTM32Data.BOLA == 0){
                R2CMotion::movingrotation(angle,mPcData,mSTM32Data);
            }
            else{
                done = false;
                hadap = false;
                not_start = true;
                R2CMotion::Motion(mPcData,0,0,0,0,2);
            }
        }
    if (mSTM32Data.BUTTON==3){
            mPcData.KICK_MODE=1;
            mPcData.TENDANG=11;
            usleep(60000);
            mPcData.KICK_MODE=0;
            mPcData.TENDANG=0;
            printf("berhasil mas toto\n");
        }
    if(mSTM32Data.BUTTON == 20){
            tes_gawang = false;
        }
        // printf("flag sekarang: %i | flag sebelumnya: %i | skill:%i | operan: %i",flagbola_current,flagbola_previous,basestationData.skillRobot,jumlah_operan);
        // printf("Catch: %i Bola Oper: %i hadap: %i  button: %i tendangndak: %i skill: %i \n",Catch,bola_oper,hadap,mSTM32Data.BUTTON,tendangbosq,basestationData.skillRobot);
        // printf("grid x: %i grid y: %i",posisixobjek,posisiyobjek);
    if(basestationData.skillRobot == 11){
       tendangbosq = true;
    }
    else{
       tendangbosq = false;
    }
    if(!giliran){
        flagbola_current = mSTM32Data.BOLA;
    }

    if(jumlah_operan<=5){
        if((!hadap) && flagbola_current == flagbola_previous){
            hadap = R2CMotion::hadap_teman(mPcData,mSTM32Data,angleFrontFriend1,angle,flagFrontFriend1,mSTM32Data.BOLA,tendangbosq);
            if(hadap){
                giliran = false;
            }
            // printf("jadi dong!!!");
        }
        else if ((flagbola_current != flagbola_previous) && (not_start) && (!Catch) && (!bola_oper)){
            if(mSTM32Data.KOMPAS<90 && mSTM32Data.KOMPAS>95){
                mPcData.VZ = (-90+mSTM32Data.KOMPAS)*3;
            }
            else{
                oper = R2CMotion::pindahgrid(gridx_setpoint,gridy_setpoint,mSTM32Data,mPcData,130,90);
            }
            if(oper){
                if((gridy_setpoint+2)<21){
                    if((gridy_setpoint+2) == posisiyobjek){
                        gridy_setpoint +=3;
                        gridx_setpoint -=3;
                        adaobjek = true;
                    }
                    else if((gridy_setpoint+1) == posisiyobjek){
                        gridy_setpoint +=1;
                        gridx_setpoint -=3;
                        adaobjek = true;
                    }
                    else{
                        gridy_setpoint+=2;    
                    }
                }
                else{
                    gridy_setpoint-=2;
                }
                jumlah_operan++;
                oper = false;
                hadap = false;
                flagbola_previous = flagbola_current;
                bola_oper = true;                
            }
        }   
    }
    else{
        tes_gawang = false;
    }
    if(mSTM32Data.BUTTON == 5){
        tes_hadap = false;
    }
    if(mSTM32Data.BUTTON == 6){
        Catch = true;
    }
    if(!tes_hadap){
        tes_hadap = R2CMotion::hadap_teman(mPcData,mSTM32Data,angleFrontFriend1,angle,flagFrontFriend1,mSTM32Data.BOLA,tendangbosq);
    }
    if(!tes_gawang){
        // tes_gawang = R2CAlgoritma::tendang_gawang(goalAngle, param_gawang, mPcData, mSTM32Data);
        tes_gawang = R2CMotion::putar_gawang(mPcData,mSTM32Data);
        if (tes_gawang){
            mPcData.VZ = 0;
            mode_1 = false;
            // reset_mode = true;
        }
            

    }
    if((hadap) && (!mSTM32Data.BOLA)&&(not_start) && (bola_oper)){
        Catch = true;
    }
    else if((hadap) && (mSTM32Data.BOLA)&&(not_start) && (bola_oper)){
        Catch = false;
        bola_oper = false;
    }
}