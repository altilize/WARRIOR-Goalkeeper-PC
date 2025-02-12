#ifndef ALGORITMA_H
#define ALGORITMA_H

#include "TypeData.h"
#include <opencv4/opencv2/opencv.hpp>

namespace R2CAlgoritma{
    double calculateOutput(float,float,float,double,double);
    bool lihat_bola_diam(bool, int, pcData&, STM32Data&);
    bool lihat_bola_geser(int,int, pcData&, STM32Data&);
    //bool putar_cari_gawang(bool&, int&, float&, int&, int&, int&, pcData&, STM32Data&);
    //bool kejar_bola_putar(int,pcData&, STM32Data&,int);
    bool oper_teman(pcData&, STM32Data&, int&, int&, bool&, bool&);
    bool tendang_gawang(int&, bool&, pcData&, STM32Data&);
    bool kejar_bola_dekat(int,pcData&, STM32Data&, int,int,int);
    bool putar_cari_gawang(bool&, bool&, int&, int&, int&, pcData&, STM32Data&);
    bool kejar_bola(bool, int, int, int, int, int, pcData&, STM32Data&);
    bool lihat_teman_diam(bool, int, pcData&, STM32Data&, bool);
    bool catch_ball(pcData&, STM32Data&, cv::Point, cv::Point, int, bool&, float,float, int);
}

#endif
