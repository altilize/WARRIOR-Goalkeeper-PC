#ifndef CITRA_H
#define CITRA_H

#include <opencv2/highgui/highgui.hpp>
#include "TypeData.h"

using namespace cv;

namespace R2CCitra
{
    Mat shifting_frame(Mat&, int&, int&);
    Mat masking_frame(Mat&, int&);
    Mat rotation_frame(Mat&, float&);
    bool init(void);
    bool read_data(void);
    void segmentasi_warna_omni(Mat&, Mat&, Mat&, Mat&, Mat&);
    void segmentasi_warna_depan(Mat&, Mat&, Mat&, Mat&);
    void convex_hull(Mat&, Mat&);
    void deteksi_bola_omni(bool&, Mat&, Mat&, float&, float&, int&, int&);
    void deteksi_gawang_depan(Mat&, Mat&, bool&, int&, int&, Point&, STM32Data&, float&);
    void deteksi_musuh_depan(Mat&, Mat&, int&, int&, bool&, bool&, Point&);
    void deteksi_musuh_omni(Mat&, Mat&, Point2f&, int&, int&, bool&);
    void deteksi_teman_omni(Mat&, Mat&, Point2f&, int&, int&, bool&);
    void deteksi_teman_depan(Mat&, Mat&, int&, int&, bool&, bool&);
    bool deteksi_center_target(Mat&, Mat&, int&, int&, bool&);
    void auto_segmentasi(Vec3b&, bool&, int&);
    void deteksi_objek_hitam(Mat&, int&, vector<int>&, vector<float>&, float&, float&, float&, STM32Data&);
    void gambar_bola_dan_heading_robot(Mat&, float&, float&, Point&, Point&, STM32Data& mSTM32Data, pcData& mPcData);
    void ball_predict(Mat&, pcData&, STM32Data&, Point&, Point&, int&, Point&, Point&);
}

#endif //CITRA_H