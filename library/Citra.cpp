#include <iostream>
#include "string.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <deque>

#include "Citra.h"
#include <fstream>

using namespace cv;
using namespace std;

/*=============VARIABEL CITRA=============*/
// Mat frame_asli;
Mat frame_HSV_omni;
Mat frame_HSV_depan;
// Mat frameLapangan, frameBola;  

Point b_current, b_previous(-1,-1), b_delta;
vector<Point>save_position;
float sumX, sumY, avgX, avgY;
float b_heading;

/*HSV*/
int gb_hmn,gb_smn,gb_vmn,gb_hmx,gb_smx,gb_vmx;	// Garis
int hb_hmn,hb_smn,hb_vmn,hb_hmx,hb_smx,hb_vmx;	// Lapangan, kamera depan
int bb_hmn,bb_smn,bb_vmn,bb_hmx,bb_smx,bb_vmx;	// Bola, kamera depan
int gw_hmn,gw_smn,gw_vmn,gw_hmx,gw_smx,gw_vmx;	// Gawang
int ogw_hmn,ogw_smn,ogw_vmn,ogw_hmx,ogw_smx,ogw_vmx;
int tb_hmn,tb_smn,tb_vmn,tb_hmx,tb_smx,tb_vmx;	// Teman, kamera depan
int to_hmn,to_smn,to_vmn,to_hmx,to_smx,to_vmx;	// Teman, kamera depan
int mb_hmn,mb_smn,mb_vmn,mb_hmx,mb_smx,mb_vmx;  // musuh depan
int mo_hmn,mo_smn,mo_vmn,mo_hmx,mo_smx,mo_vmx;  // musuh depan

int ho_hmn,ho_smn,ho_vmn,ho_hmx,ho_smx,ho_vmx;	// Lapangan, kamera omni
int bo_hmn,bo_smn,bo_vmn,bo_hmx,bo_smx,bo_vmx;	// Bola, kamera omni

int counterBola = 0;
const int radius_min = 0;
const int radius_max = 500;
const float kesamaan = 0.01;
const float orangeRatio = 0.01;
bool tungguBola = true;

vector<Point> convexHullPoints;
vector<Point> hor;
Point gawang_kanan;
Point gawang_kiri;

/*====================KALMAN FILTER===================*/
class KalmanFilter 
{
    public:
        KalmanFilter(float mea_e, float est_e, float q);
        float updateEstimate(float mea);
        void setMeasurementError(float mea_e);
        void setEstimateError(float est_e);
        void setProcessNoise(float q);
        float getKalmanGain();
  
    private:
        float _err_measure;
        float _err_estimate;
        float _q;
        float _current_estimate;
        float _last_estimate;
        float _kalman_gain;
};

KalmanFilter::KalmanFilter(float mea_e, float est_e, float q){
  _err_measure=mea_e;
  _err_estimate=est_e;
  _q = q;
}

float KalmanFilter::updateEstimate(float mea){
  _kalman_gain = _err_estimate/(_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimate-_current_estimate)*_q;
  _last_estimate=_current_estimate;

  return _current_estimate;
}

void KalmanFilter::setMeasurementError(float mea_e){
  _err_measure=mea_e;
}

void KalmanFilter::setEstimateError(float est_e){
  _err_estimate=est_e;
}

void KalmanFilter::setProcessNoise(float q){
  _q=q;
}

float KalmanFilter::getKalmanGain() {
  return _kalman_gain;
}

/*=============PARAMETER KALMAN FILTER================*/
KalmanFilter koordX_bola(0.5,2,0.2);
KalmanFilter koordY_bola(0.5,2,0.2);
KalmanFilter radiusBola(0.5,1,0.1);

KalmanFilter koordX_musuh1(0.5,2,0.2);
KalmanFilter koordX_musuh2(0.5,2,0.2);
KalmanFilter koordX_musuh3(0.5,2,0.2);
KalmanFilter koordY_musuh1(0.5,2,0.2);
KalmanFilter koordY_musuh2(0.5,2,0.2);
KalmanFilter koordY_musuh3(0.5,2,0.2);

KalmanFilter koordX_teman1(0.5,2,0.2);
KalmanFilter koordX_teman2(0.5,2,0.2);
KalmanFilter koordY_teman1(0.5,2,0.2);
KalmanFilter koordY_teman2(0.5,2,0.2);

KalmanFilter radiusTeman(0.5,1,0.1);
KalmanFilter radiusMusuh(0.5,1,0.1);

KalmanFilter koordX_gawang(0.5,1,0.1);
KalmanFilter koordY_gawang(0.5,1,0.1);
KalmanFilter min_gawang(0.5,1,0.1);
KalmanFilter max_gawang(0.5,1,0.1);
KalmanFilter rerata2_gawang(0.5,1,0.1);
KalmanFilter rerata1_gawang(0.5,1,0.1);

KalmanFilter koordX_temanFront(0.5,1,0.1);
KalmanFilter koordY_temanFront(0.5,1,0.1);
//KalmanFilter radiusBola(1,1,0.03);
/*=====================READ DATA===================*/

bool R2CCitra::read_data(){
	FILE* input;
	int count = 0;
	
	input = fopen("kalibrasi/lapangan.txt","r");
	if(input==NULL)	{
		perror("kalibrasi/lapangan.txt");
		count++;
	}
	perror("kalibrasi/lapangan.txt");
	fscanf(input,"%i\n%i\n%i\n%i\n%i\n%i\n",&hb_hmn,&hb_smn,&hb_vmn,&hb_hmx,&hb_smx,&hb_vmx);
	fclose(input);

	input = fopen("kalibrasi/bola.txt","r");
	if(input==NULL)	{
		perror("kalibrasi/bola.txt");
		count++;
	}
	perror("kalibrasi/bola.txt");
	fscanf(input,"%i\n%i\n%i\n%i\n%i\n%i\n",&bb_hmn,&bb_smn,&bb_vmn,&bb_hmx,&bb_smx,&bb_vmx);
	fclose(input);

	input = fopen("kalibrasi/musuh_depan.txt","r");
	if(input==NULL)	{
		perror("kalibrasi/musuh_depan.txt");
		count++;
	}
	perror("kalibrasi/musuh_depan.txt");
	fscanf(input,"%i\n%i\n%i\n%i\n%i\n%i\n",&mb_hmn,&mb_smn,&mb_vmn,&mb_hmx,&mb_smx,&mb_vmx);
	fclose(input);

    input = fopen("kalibrasi/teman.txt","r");
	if(input==NULL)	{
		perror("kalibrasi/teman.txt");
		count++;
	}
	perror("kalibrasi/teman.txt");
	fscanf(input,"%i\n%i\n%i\n%i\n%i\n%i\n",&to_hmn,&to_smn,&to_vmn,&to_hmx,&to_smx,&to_vmx);
	fclose(input);

	input = fopen("kalibrasi/teman_depan.txt","r");
	if(input==NULL)	{
		perror("kalibrasi/teman_depan.txt");
		count++;
	}
	perror("kalibrasi/teman_depan.txt");
	fscanf(input,"%i\n%i\n%i\n%i\n%i\n%i\n",&tb_hmn,&tb_smn,&tb_vmn,&tb_hmx,&tb_smx,&tb_vmx);
	fclose(input);

	/*
		Data gawang hanya digunakan hanya untuk kamera depan
	*/
	input = fopen("kalibrasi/gawang_depan.txt","r");
	if(input==NULL)	{
		perror("kalibrasi/gawang_depan.txt");
		count++;
	}
	perror("kalibrasi/gawang_depan.txt");
	fscanf(input,"%i\n%i\n%i\n%i\n%i\n%i\n",&gw_hmn,&gw_smn,&gw_vmn,&gw_hmx,&gw_smx,&gw_vmx);
	fclose(input);

	input = fopen("kalibrasi/musuh.txt","r");
	if(input==NULL)	{
		perror("kalibrasi/musuh.txt");
		count++;
	}
	
	perror("kalibrasi/musuh.txt");
	fscanf(input,"%i\n%i\n%i\n%i\n%i\n%i\n",&mo_hmn,&mo_smn,&mo_vmn,&mo_hmx,&mo_smx,&mo_vmx);
	fclose(input);

	// if(count > 0)return false;
	return true;
}

Mat R2CCitra::shifting_frame(Mat& frame, int& dx, int& dy){
    Mat frame_shifted = cv::Mat::zeros(frame.rows, frame.cols, frame.type());

    int x_start = max(0, dx);
    int y_start = max(0, dy);
    int x_end = min(frame.cols, frame.cols + dx);
    int y_end = min(frame.rows, frame.rows + dy);

    frame(cv::Rect(x_start, y_start, x_end - x_start, y_end - y_start)).copyTo(
        frame_shifted(cv::Rect(x_start - dx, y_start - dy, x_end - x_start, y_end - y_start))
    );

    return frame_shifted;
}

Mat R2CCitra::masking_frame(Mat& frame, int& radius_frame){
    Point center_frame(frame.cols / 2, frame.rows / 2);
    Mat frame_temp = Mat::zeros(frame.size(), CV_8UC1);
    circle(frame_temp, center_frame, radius_frame, Scalar(255), -1);

    Mat frame_mask;
    bitwise_and(frame, frame, frame_mask, frame_temp);

    return frame_mask;
}

Mat R2CCitra::rotation_frame(Mat& frame, float& sudut_putaran_frame){
    Point center_frame(frame.cols / 2, frame.rows / 2);
    Mat matrix = getRotationMatrix2D(center_frame, sudut_putaran_frame, 1);

    Mat frame_rotation;
    warpAffine(frame, frame_rotation, matrix, frame.size());

    return frame_rotation;
}

/*========================SEGMENTASI WARNA===================*/
void R2CCitra::segmentasi_warna_omni(Mat& frame_asli_omni, Mat& frameBola, Mat&frameLapangan, Mat&frameMusuh, Mat& frameTeman){
	//GaussianBlur(frame_asli_omni, frame_asli_omni, Size(1,1), 0);
    //medianBlur(frame_asli_omni,frame_asli_omni, 5);
	cvtColor(frame_asli_omni, frame_HSV_omni, COLOR_BGR2HSV);
	inRange(frame_HSV_omni, Scalar(hb_hmn, hb_smn, hb_vmn), Scalar(hb_hmx, hb_smx, hb_vmx), frameLapangan);
    inRange(frame_HSV_omni, Scalar(bb_hmn, bb_smn, bb_vmn), Scalar(bb_hmx, bb_smx, bb_vmx), frameBola);
    inRange(frame_HSV_omni, Scalar(to_hmn, to_smn, to_vmn), Scalar(to_hmx, to_smx, to_vmx), frameTeman);
    inRange(frame_HSV_omni, Scalar(mo_hmn, mo_smn, mo_vmn), Scalar(mo_hmx, mo_smx, mo_vmx), frameMusuh);
    
    //imshow("Segmentasi Bola", frameBola);
    //imshow("Segmentasi Lapangan", frameLapangan);
    //imshow("Segmentasi teman", frameTeman);
}

void R2CCitra::segmentasi_warna_depan(Mat& frame_asli_depan, Mat& frameGawangDepan, Mat& frameTemanDepan, Mat& frameMusuhDepan){
	GaussianBlur(frame_asli_depan, frame_asli_depan, Size(7,7), 0);
	cvtColor(frame_asli_depan, frame_HSV_depan, COLOR_BGR2HSV);
    inRange(frame_HSV_depan, Scalar(gw_hmn, gw_smn, gw_vmn), Scalar(gw_hmx, gw_smx, gw_vmx), frameGawangDepan);
    inRange(frame_HSV_depan, Scalar(tb_hmn, tb_smn, tb_vmn), Scalar(tb_hmx, tb_smx, tb_vmx), frameTemanDepan);
    inRange(frame_HSV_depan, Scalar(mb_hmn, mb_smn, mb_vmn), Scalar(mb_hmx, mb_smx, mb_vmx), frameMusuhDepan);
	//imshow("Segmentasi Gawang", frameGawang);
    //imshow("Segmentasi teman/musuh", frameMusuh);
}

/*=============================GRAHAM SCAN=============================*/
// Fungsi untuk menentukan orientasi tiga titik (orientasi antara tiga titik)
int orientation(Point p, Point q, Point r) {
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0; // lurus
    return (val > 0) ? 1 : 2; // searah jarum jam atau berlawanan arah jarum jam
}

// Fungsi untuk membandingkan dua titik berdasarkan sudut terhadap titik pivot
bool comparePoints(Point p1, Point p2, Point pivot) {
    int o = orientation(pivot, p1, p2);
    if (o == 0) {
        return (pow(pivot.x - p2.x, 2) + pow(pivot.y - p2.y, 2) >= pow(pivot.x - p1.x, 2) + pow(pivot.y - p1.y, 2));
    }
    return (o == 2); // Jika p2 berada dalam arah berlawanan jarum jam dengan p1, maka p1 < p2
}

void R2CCitra::convex_hull(Mat& frame_asli, Mat& frameLapangan) {
    vector<vector<Point>> contours;

    Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(15, 15));
    Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(10, 10));

    dilate(frameLapangan, frameLapangan, kernel_dilate);
    erode(frameLapangan, frameLapangan, kernel_erode);

    findContours(frameLapangan, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Cari kontur dengan luas terbesar
    int largest_contour_index = -1;
    double largest_area = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > largest_area) {
            largest_area = area;
            largest_contour_index = i;
        }
    }

    // Hapus kontur yang lebih kecil dari kontur terbesar
    for (size_t i = 0; i < contours.size(); i++) {
        if (i != largest_contour_index) {
            contours[i].clear();
        }
    }

    // Convex Hull menggunakan algoritma Graham Scan
    if (contours.size() > 0) {
        vector<Point> hull_points;
        vector<Point>& contour = contours[largest_contour_index];

        // Cari pivot (titik dengan nilai y terkecil)
        int pivot_idx = 0;
        for (size_t i = 1; i < contour.size(); i++) {
            if (contour[i].y < contour[pivot_idx].y || (contour[i].y == contour[pivot_idx].y && contour[i].x < contour[pivot_idx].x)) {
                pivot_idx = i;
            }
        }

        // Urutkan titik-titik berdasarkan sudut terhadap titik pivot
        swap(contour[0], contour[pivot_idx]);
        sort(contour.begin() + 1, contour.end(), [contour](Point p1, Point p2) {
            return comparePoints(p1, p2, contour[0]);
        });

        // Push pivot dan dua titik pertama ke dalam hull
        hull_points.push_back(contour[0]);
        hull_points.push_back(contour[1]);

        // Proses sisa titik
        for (size_t i = 2; i < contour.size(); i++) {
            while (hull_points.size() > 1 && orientation(hull_points[hull_points.size() - 2], hull_points[hull_points.size() - 1], contour[i]) != 2) {
                hull_points.pop_back();
            }
            hull_points.push_back(contour[i]);
        }

        // Gambar Convex Hull
        vector<vector<Point>> hull = { hull_points };
		convexHullPoints = hull_points;

		//imshow("Lapangan", frameLapangan);
        drawContours(frame_asli, hull, 0, Scalar(0, 0, 255), 1);
    }
}

int last_koord_x = -1;
int last_koord_y = -1;

void R2CCitra::deteksi_bola_omni(bool &flag_bola, Mat& frame_asli, Mat& frameBola, float &sudut, float &jarak, int &koord_x, int &koord_y){
	Point2f ball_center;
	float ball_radius;
	int orange_pix = 0;
	float orange_ratio = 0,sum_now = 0,sum_max = 0;
	float radiusPatokan = 0;
	float ratioRadius = 0;

    // int centerX = frame_asli.cols/2;
    // int centerY = frame_asli.rows/2;

    int centerX = 321;
    int centerY = 250;

    vector<vector<Point>> contours;

    medianBlur(frameBola, frameBola, 1);

    //operasi morfologi
	// dilate(frameBola,frameBola,getStructuringElement(MORPH_ELLIPSE,Size(2,2)));
	// erode(frameBola,frameBola,getStructuringElement(MORPH_ELLIPSE,Size(2,2)));
    // erode(frameBola,frameBola,getStructuringElement(MORPH_ELLIPSE,Size(2,2)));
	// dilate(frameBola,frameBola,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));

    // dilate(frameBola,frameBola,getStructuringElement(MORPH_ELLIPSE,Size(10,10)));
	// erode(frameBola,frameBola,getStructuringElement(MORPH_ELLIPSE,Size(2,2)));

    dilate(frameBola, frameBola, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(frameBola, frameBola, getStructuringElement(MORPH_ELLIPSE, Size(1,1)));

    erode(frameBola, frameBola, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(frameBola, frameBola, getStructuringElement(MORPH_ELLIPSE, Size(20,20)));

	Canny (frameBola, frameBola, 100, 100*2, 5);

    findContours(frameBola, contours, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0,0));

    vector<float> radius (contours.size());
    vector<Point2f> center (contours.size());
	vector<Moments> mu (contours.size());
	vector<Point2f> mc (contours.size());
    
    line(frame_asli, Point(centerX,0),Point(centerX,centerY),Scalar(50,255,0),1,8);
	
    for(int i = 0; i < contours.size(); i++)
	{
		minEnclosingCircle(Mat(contours[i]),center[i],radius[i]	);
		// circle( frame, center[i], radius[i], Scalar( 255, 0, 0 ), 1, 8 );
		if(radius[i] > radius_min && radius[i] < radius_max)
		{
			float selisih_x = 0, selisih_y = 0, selisih_r = 0, circle_area = 0, object_area = 0, similarity = 0;
			int x1 = center[i].x - radius[i];
			int x2 = center[i].x + radius[i];
			int y1 = center[i].y - radius[i];
			int y2 = center[i].y + radius[i];
			
			mu[i] = moments(contours[i],false);
			mc[i] = Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
			selisih_x = abs(center[i].x - mc[i].x);
			selisih_y = abs(center[i].y - mc[i].y);
			selisih_r = sqrt((selisih_x*selisih_x)+(selisih_y*selisih_y));
			selisih_r = radius[i] - selisih_r;
			circle_area = M_PI * radius[i] * radius[i];
			object_area = M_PI * selisih_r * selisih_r;
			similarity = object_area / circle_area;
			orange_pix=0;
			for (int y = y1 ; y < y2 ; y++)
			{
				for (int x = x1; x < x2 ; x++ )
				{
					if(x<0 || y<0 || x>frameBola.cols || y>frameBola.rows){
						continue;
					}
					else
					{
						if (frameBola.at<uchar>(y,x) > 0)
						{
							orange_pix++;
						}
					}
				}
			}
			orange_ratio = (float)orange_pix/circle_area;

			if(similarity >= kesamaan && orange_ratio >= orangeRatio)// && ratioRadius > 0.7 && ratioRadius < 1.3)
			{
				sum_now = circle_area;	
				if(sum_now > sum_max)
				{
					sum_max = sum_now;
					ball_center = center[i];
					ball_radius = radius[i];
					flag_bola = true;
				}
			}
		}
	}
	if(flag_bola)
	{
		tungguBola = true;
		counterBola++;
		if(counterBola > 5 || tungguBola == true){
			koord_x = koordX_bola.updateEstimate((int)ball_center.x); //uncommand untuk aktifkan kalman filter deteksi bola
			koord_y = koordY_bola.updateEstimate((int)ball_center.y); //uncimmand untuk aktifkan kalman filter deteksi bola
            Point ballCenter(koord_x, koord_y);
            float RadiusBola = radiusBola.updateEstimate((float)ball_radius);
            
            double distanceHull = pointPolygonTest(convexHullPoints, ballCenter, true);
            if (distanceHull <= 0){
				sudut = 0;
				jarak = 0;
				//koord_x = 0;
				//koord_y = 0;
                flag_bola = false;
            }
			else{
                stringstream label;
                circle( frame_asli, ballCenter, RadiusBola, Scalar( 255, 0, 0 ), 1, 8 );
                circle( frame_asli, ballCenter, 1, Scalar( 255, 0, 0 ), 3, 8 );
                label << "Bola"; // Label sesuai dengan urutan deteksi
                putText(frame_asli, label.str(), Point(ball_center.x - 30, ball_center.y + 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
            
                line (frame_asli, Point(centerX, centerY), Point(koord_x,koord_y),Scalar(0,0,255),1,8);
                
                sudut = -1*atan2(centerX - koord_x, centerY - koord_y) * 180 / M_PI;

                float dx = koord_x - centerX;
                float dy = koord_y - centerY;
                jarak = sqrt((dx*dx) + (dy*dy));
                flag_bola = true;

                last_koord_x = koord_x;
                last_koord_y = koord_y;
			}
            //printf("P(x,y,theta) = %i, %i, %i | jarak bola (pxl) = %.2f\n", koord_x, koord_y, (int) sudut, jarak);
			//cout << "Sudut bola: " << (int)sudut << " derajat" << endl;
        	//cout << "Jarak bola: " << (int)jarak << endl;
            //printf("Jarak bola (pxl) = %.2f\n", jarak);
            //cout << "Koordinat bola terpilih (Pixel): x = " << koord_x << ", y = " << koord_y << endl;
		}
	}
	else
	{
		sudut = 0;
        jarak = 0;
        //koord_x = 0;
        //koord_y = 0;
        flag_bola = false;

        // Gunakan koordinat terakhir jika bola tidak terdeteksi
        if (last_koord_x != -1 && last_koord_y != -1) {
            koord_x = last_koord_x;
            koord_y = last_koord_y;
        }
	}
    imshow("frame1", frameBola);
}

void R2CCitra::deteksi_gawang_depan(Mat& frame_asli, Mat& frameGawangDepan, bool& flagAdaMistar, int& sudutGawang, int& sudutMusuhKiper, Point& centerEnemyKiper, STM32Data& mSTM32Data, float& dummyPosition)
{
	Mat edge_gawang;
	vector<Vec4i> lines_gawang;
	vector<Vec4i> horizontal;
	vector<Vec4i> mistar;
	float derajat;
	float perbandingan;
	flagAdaMistar=false;
	int min=999;
	int max=0;
	int sum1 = 0;
	int sum2 = 0;
	int counter = 0;
	float rerata1 = 0;
	float rerata2 = 0;
    int centerX = frame_asli.cols/2;
    int centerY = frame_asli.rows/2;

	dilate(frameGawangDepan,frameGawangDepan,getStructuringElement(MORPH_RECT,Size(5,5)));
	erode(frameGawangDepan,frameGawangDepan,getStructuringElement(MORPH_RECT,Size(7,7)));
    Canny(frameGawangDepan,edge_gawang,100,300,5);
    GaussianBlur(edge_gawang,edge_gawang,Size(5,5),0,0);
	HoughLinesP(edge_gawang,lines_gawang,1,(1*CV_PI/180),80,120,5);

    line(frame_asli, Point (320,0), Point (320,480), Scalar(0, 255, 0), 1);
//	printf("%i=\n", lines_gawang.size());
	for(int i = 0; i < lines_gawang.size();i++)
	{
		Vec4i l = lines_gawang[i];
		derajat = atan(((float)l[3]-(float)l[1])/((float)l[2]-(float)l[0])) * 180 / M_PI;

		if(abs(derajat) < 20)
		{
			Vec4i l = lines_gawang[i];
			if(l[1] < 300 || l[3] < 300)
			{
				line(frame_asli,Point((int)l[0],(int)l[1]),Point((int)l[2],(int)l[3]),Scalar(0,255,0),3,8);	
				horizontal.push_back(l);
			}
		}
		// line(frame,Point((int)l[0],(int)l[1]),Point((int)l[2],(int)l[3]),Scalar(0,255,0),3,8);	
	}
	for(int i = 0;i < horizontal.size();i++)
	{
		Vec4i l = horizontal[i];
		for(int j = 0;j<horizontal.size();j++)
		{
			if(j == i)
			{
				continue;
			}
			Vec4i k = horizontal[j];
			perbandingan = (float)abs(l[0]-k[0])/(float)abs(l[3]-l[1]);
			//printf("%.2f\n", perbandingan);
			if(perbandingan >= 0.10 && perbandingan <= 20 || perbandingan == INFINITY)//&& abs(l[1]-k[1]) > 10 && abs(l[3]-k[3]) > 10 && abs(l[0]-k[0]) < 200 && abs(l[2]-k[2]) < 200)// && (abs(l[1]-k[1]) < 100 || abs(l[3]-k[3]) < 100))
			{
				//line(frame,Point((int)l[0],(int)l[1]),Point((int)l[2],(int)l[3]),Scalar(255,0,0),3,8);
				//line(frame,Point((int)k[0],(int)k[1]),Point((int)k[2],(int)k[3]),Scalar(255,0,0),3,8);
				if(l[0] < min)
				{
					min = l[0];
				}
				if(k[0] < min)
				{
					min = k[0];
				}
				if(l[2] > max)
				{
					max = l[2];
				}
				if(k[2] > max)
				{
					max = k[2];
				}
				sum1 += l[1];
				sum1 += k[1];
				sum2 += l[3];
				sum2 += k[3];
				counter+=2;
				flagAdaMistar = true;
			}
		}
	}

    if(mSTM32Data.KOMPAS >= 155 && mSTM32Data.KOMPAS <= -155)
    {
        flagAdaMistar = false;
    }

	if(flagAdaMistar)
	{
		rerata1 = (float)sum1/(float)counter;
		rerata2 = (float)sum1/(float)counter;
        //cout << (int)rerata1 << "|" << (int)rerata2 << endl;

        int min_ = min_gawang.updateEstimate(min);
        int max_ = max_gawang.updateEstimate(max);

        int rerata1_ = rerata1_gawang.updateEstimate(rerata1);
        int rerata2_ = rerata2_gawang.updateEstimate(rerata2);


        // Menghitung titik tengah dari garis gawang
        int mid_x = (min + max) / 2;
        int mid_y = (rerata1 + rerata2) / 2;

        int mid_x_ = koordX_gawang.updateEstimate(mid_x);
        int mid_y_ = koordY_gawang.updateEstimate(mid_y);
        gawang_kanan = Point((int)min_, (int)rerata1_);
        gawang_kiri = Point((int)max_, (int)rerata2_);
        // Menggambar titik tengah pada gambar
        circle(frame_asli, Point((int)min_, (int)rerata1_), 5, Scalar(255, 0, 0), -1);
        circle(frame_asli, Point((int)max_, (int)rerata2_), 5, Scalar(255, 0, 0), -1);
        circle(frame_asli, Point(mid_x_, mid_y_), 5, Scalar(255, 0, 0), -1);

        int sudutGawangKiri = -1*atan2(320- min_ , 480 - rerata1_) * 180 / CV_PI;
        int sudutGawangKanan = -1*atan2(320- max_ , 480 - rerata2_) * 180 / CV_PI;

		line(frame_asli,Point((int)min_,(int)rerata1_),Point((int)max_,(int)rerata2_),Scalar(0,0,255),3,8);
        //line(frame_asli, Point(mid_x_, mid_y_), Point (320,480), Scalar(255,255,0),1,8);

        if((min_ + mid_x) > 900 && (min_ + mid_x) < 1200)
        {
            flagAdaMistar = false;
        }

        if((max_ + mid_x) > 70 && (max_ + mid_x) < 200)
        {
            flagAdaMistar = false;
        }
        cout << dummyPosition << endl;
        if ((dummyPosition >= min_ && dummyPosition <= mid_x_)) //(centerEnemyKiper.x >= min_ && centerEnemyKiper.x <= mid_x_) ||
        {
            circle(frame_asli, Point((max_ + mid_x_)/2, (mid_y_)), 5, Scalar(255, 255, 0), -1);
            line(frame_asli, Point((max_ + mid_x_)/2, mid_y_), Point (320,480), Scalar(255,255,0),1,8);
            sudutGawang = -1*atan2(320- (max_ + mid_x_ )/2, 480 - mid_y_) * 180 / CV_PI;
        }
        else if( (dummyPosition >= mid_x_ && dummyPosition <= max_)) //(centerEnemyKiper.x >= mid_x_ && centerEnemyKiper.x <= max_) ||
        {
            circle(frame_asli, Point((min_ + mid_x_)/2, (mid_y_)), 5, Scalar(255, 255, 0), -1);
            line(frame_asli, Point((min_ + mid_x_)/2, mid_y_), Point (320,480), Scalar(255,255,0),1,8);
            sudutGawang = -1*atan2(320- (min_ + mid_x_ )/2, 480 - mid_y_) * 180 / CV_PI;
        }
        else
        {
            circle(frame_asli, Point((mid_x), (mid_y_)), 5, Scalar(255, 255, 0), -1);
            line(frame_asli, Point(mid_x_, mid_y_), Point (320,480), Scalar(255,255,0),1,8);
            sudutGawang = -1*atan2(320- mid_x_ , 480 - mid_y_) * 180 / CV_PI;
        }
	}
	else
	{
        sudutGawang = 0;
    }

    imshow("tes",frameGawangDepan);  
}

void R2CCitra::deteksi_teman_depan(Mat& frame_asli, Mat& frameTemanDepan, int& sudutTeman1, int& sudutTeman2, bool& flag_friend1, bool& flag_friend2)
{
    int sudutTeman;
    int frameWidth = frame_asli.cols;
    int frameHeight = frame_asli.rows;
    flag_friend1 = false;
    flag_friend2 = false;
    Point centerFrame(frameWidth / 2, frameHeight); 
    vector<vector<Point>> contours;
    vector<Rect> boundingBoxes;
    vector<Point> centers;
    Mat morphElement1 = getStructuringElement(MORPH_RECT, Size(3,3));
    Mat morphElement2 = getStructuringElement(MORPH_RECT, Size(5,5));

    morphologyEx(frameTemanDepan, frameTemanDepan, MORPH_CLOSE, morphElement1);
    morphologyEx(frameTemanDepan, frameTemanDepan, MORPH_OPEN, morphElement2);
    findContours(frameTemanDepan, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) 
    {
        double area = contourArea(contour);
        if (area > 100) 
        { 
            Rect boundingBox = boundingRect(contour);
            boundingBoxes.push_back(boundingBox);
            Point centerBox(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
            centers.push_back(centerBox);
        }
    }

    sort(boundingBoxes.begin(), boundingBoxes.end(), [](const Rect& a, const Rect& b) { return a.x < b.x; });
    sort(centers.begin(), centers.end(), [](const Point& a, const Point& b) { return a.x < b.x; });

    for (size_t i = 0; i < boundingBoxes.size(); i++) {
        Rect boundingBox = boundingBoxes[i];
        Point centerBox = centers[i];

        int koord_x = koordX_temanFront.updateEstimate(centerBox.x);
        int koord_y = koordY_temanFront.updateEstimate(centerBox.y);

        rectangle(frame_asli, boundingBox, Scalar(0, 255, 0), 2);
        line(frame_asli, centerFrame, Point(koord_x, koord_y), Scalar(0, 0, 255), 1);
        sudutTeman = atan2(koord_x - frameWidth / 2, frameHeight - koord_y) * 180 / CV_PI;

        if (i == 0) {
            sudutTeman1 = sudutTeman;
            flag_friend1 = true;
        } else if (i == 1) {
            flag_friend2 = true;
            sudutTeman2 = sudutTeman;
        }

        stringstream ss;
        ss << "Sudut: " << sudutTeman;
        putText(frame_asli, ss.str(), Point(boundingBox.x, boundingBox.y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
    }
    //imshow("teman", frameTemanDepan);
}

void R2CCitra::deteksi_teman_omni(Mat& frame_asli, Mat& frameTeman, Point2f& center_teman1, int& sudutTeman1, int& jarak1, bool& flag_teman) {
    int erodeSize = 2;  // Ukuran elemen struktur untuk erosi
    int dilateSize = 9; // Ukuran elemen struktur untuk dilasi
    int erodeIterations = 2;  // Jumlah iterasi erosi
    int dilateIterations = 2; // Jumlah iterasi dilasi

    center_teman1.x = 0.0; center_teman1.y = 0.0;

    sudutTeman1 = 0.0;

    int centerX = frame_asli.cols/2;
    int centerY = frame_asli.rows/2;

    // bool flag_teman1 = false;
    // bool flag_teman2 = false;
    // Closing (erosi diikuti oleh dilasi)
    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(erodeSize, erodeSize));
    erode(frameTeman, frameTeman, erodeElement, Point(-1, -1), erodeIterations);

    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(dilateSize, dilateSize));
    dilate(frameTeman, frameTeman, dilateElement, Point(-1, -1), dilateIterations);

    // Opening (dilasi diikuti oleh erosi)
    dilate(frameTeman, frameTeman, dilateElement, Point(-1, -1), dilateIterations);
    erode(frameTeman, frameTeman, erodeElement, Point(-1, -1), erodeIterations);

    medianBlur(frameTeman, frameTeman, 3);

    vector<vector<Point>> contours;
    findContours(frameTeman, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        // Cari kontur dengan luas terbesar
        auto largestContour = max_element(contours.begin(), contours.end(),
            [](const vector<Point>& a, const vector<Point>& b) {
                return contourArea(a) < contourArea(b);
            });

        // Ambil luas kontur terbesar
        double largestContourArea = contourArea(*largestContour);

        // Hapus area yang lebih kecil dari 10% luas kontur terbesar
        double minContourArea = 0.1 * largestContourArea;

        size_t maxTeman = std::min(static_cast<size_t>(2), contours.size());

        for (size_t i = 0; i < maxTeman; i++) {
            double area = contourArea(contours[i]);
            if (area > minContourArea) {
                Point2f center;
                float radius;
                minEnclosingCircle(contours[i], center, radius);
                //flag_teman1 = true;
                center_teman1.x = koordX_teman1.updateEstimate((int)center.x);
                center_teman1.y = koordY_teman1.updateEstimate((int)center.y);
                float radius_teman1 = radiusTeman.updateEstimate(radius);

                circle(frame_asli, center_teman1, radius_teman1, Scalar(0, 255, 255), 2);

                float dx1 = center_teman1.x - centerX;
                float dy1 = center_teman1.y - centerY;
                jarak1 = sqrt((dx1*dx1) + (dy1*dy1));

                sudutTeman1 = -1*atan2(centerX - center_teman1.x, centerY - center_teman1.y) * 180 / M_PI;

                stringstream label1;
                label1 << "Teman" << "1";
                putText(frame_asli, label1.str(), Point(center_teman1.x - 37, center_teman1.y + 37), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
                flag_teman = true;
            }
        }
    } 
    else
    {
        sudutTeman1 = 0.0;
        flag_teman = false;
    }

    imshow("Teman", frameTeman);
    
    //cout << (int)sudutTeman1 << "|" << (int)sudutTeman2 << endl;
}

void R2CCitra::deteksi_musuh_depan(Mat& frame_asli, Mat& frameMusuhDepan, int& sudutMusuh1, int& sudutMusuh2, bool& flag_enemy1, bool& flag_enemy2, Point& centerEnemyFront) {
    int sudutMusuh;
    int frameWidth = frame_asli.cols;
    int frameHeight = frame_asli.rows;
    flag_enemy1 = false;
    flag_enemy2 = false;
    Point centerFrame(frameWidth / 2, frameHeight); 
    vector<vector<Point>> contours;
    vector<Rect> boundingBoxes;
    vector<Point> centers;
    Mat morphElement1 = getStructuringElement(MORPH_RECT, Size(3,3));
    Mat morphElement2 = getStructuringElement(MORPH_RECT, Size(5,5));

    morphologyEx(frameMusuhDepan, frameMusuhDepan, MORPH_CLOSE, morphElement1);
    morphologyEx(frameMusuhDepan, frameMusuhDepan, MORPH_OPEN, morphElement2);
    findContours(frameMusuhDepan, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) 
    {
        double area = contourArea(contour);
        if (area > 100) 
        { 
            Rect boundingBox = boundingRect(contour);
            boundingBoxes.push_back(boundingBox);
            Point centerBox(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
            centers.push_back(centerBox);
        }
    }

    sort(boundingBoxes.begin(), boundingBoxes.end(), [](const Rect& a, const Rect& b) { return a.x < b.x; });
    sort(centers.begin(), centers.end(), [](const Point& a, const Point& b) { return a.x < b.x; });

    for (size_t i = 0; i < boundingBoxes.size(); i++) {
        Rect boundingBox = boundingBoxes[i];
        Point centerBox = centers[i];

        centerEnemyFront.x = koordX_temanFront.updateEstimate(centerBox.x);
        centerEnemyFront.y = koordY_temanFront.updateEstimate(centerBox.y);
        //printf("%i|%i\n", koord_x, koord_y);
        //cout << centerEnemyFront << endl;

        rectangle(frame_asli, boundingBox, Scalar(255, 255, 0), 2);
        line(frame_asli, centerFrame, Point(centerEnemyFront.x, centerEnemyFront.y), Scalar(255, 0, 255), 1);
        sudutMusuh = atan2(centerEnemyFront.x - frameWidth / 2, frameHeight - centerEnemyFront.y) * 180 / CV_PI;

        if (i == 0) {
            flag_enemy1 = true;
            sudutMusuh1 = sudutMusuh;
        } else if (i == 1) {
            flag_enemy1 = true;
            sudutMusuh2 = sudutMusuh;
        }

        stringstream ss;
        ss << "Sudut: " << sudutMusuh;
        putText(frame_asli, ss.str(), Point(boundingBox.x, boundingBox.y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
    }
    //imshow("teman", frameTemanDepan);
}

void R2CCitra::deteksi_musuh_omni(Mat& frame_asli, Mat& frameMusuh, Point2f& center_musuh1, int& sudutMusuh1, int& jarak1, bool& flag_musuh) {
    int erodeSize = 2;  // Ukuran elemen struktur untuk erosi
    int dilateSize = 7; // Ukuran elemen struktur untuk dilasi
    int erodeIterations = 1;  // Jumlah iterasi erosi
    int dilateIterations = 2; // Jumlah iterasi dilasi

    center_musuh1.x = 0.0; center_musuh1.y = 0.0;

    sudutMusuh1 = 0.0;

    int centerX = frame_asli.cols/2;
    int centerY = frame_asli.rows/2;

    // bool flag_teman1 = false;
    // bool flag_teman2 = false;
    // Closing (erosi diikuti oleh dilasi)
    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(erodeSize, erodeSize));
    erode(frameMusuh, frameMusuh, erodeElement, Point(-1, -1), erodeIterations);

    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(dilateSize, dilateSize));
    dilate(frameMusuh, frameMusuh, dilateElement, Point(-1, -1), dilateIterations);

    // Opening (dilasi diikuti oleh erosi)
    dilate(frameMusuh, frameMusuh, dilateElement, Point(-1, -1), dilateIterations);
    erode(frameMusuh, frameMusuh, erodeElement, Point(-1, -1), erodeIterations);

    medianBlur(frameMusuh, frameMusuh, 3);

    vector<vector<Point>> contours;
    findContours(frameMusuh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        // Cari kontur dengan luas terbesar
        auto largestContour = max_element(contours.begin(), contours.end(),
            [](const vector<Point>& a, const vector<Point>& b) {
                return contourArea(a) < contourArea(b);
            });

        // Ambil luas kontur terbesar
        double largestContourArea = contourArea(*largestContour);

        // Hapus area yang lebih kecil dari 10% luas kontur terbesar
        double minContourArea = 0.1 * largestContourArea;

        size_t maxTeman = std::min(static_cast<size_t>(2), contours.size());

        for (size_t i = 0; i < maxTeman; i++) {
            double area = contourArea(contours[i]);
            if (area > minContourArea) {
                Point2f center;
                float radius;
                minEnclosingCircle(contours[i], center, radius);
                //flag_teman1 = true;
                center_musuh1.x = koordX_teman1.updateEstimate((int)center.x);
                center_musuh1.y = koordY_teman1.updateEstimate((int)center.y);
                float radius_musuh1 = radiusMusuh.updateEstimate(radius);

                circle(frame_asli, center_musuh1, radius_musuh1, Scalar(0, 255, 255), 2);

                float dx1 = center_musuh1.x - centerX;
                float dy1 = center_musuh1.y - centerY;
                jarak1 = sqrt((dx1*dx1) + (dy1*dy1));

                sudutMusuh1 = -1*atan2(centerX - center_musuh1.x, centerY - center_musuh1.y) * 180 / M_PI;

                stringstream label1;
                label1 << "Teman" << "1";
                putText(frame_asli, label1.str(), Point(center_musuh1.x - 37, center_musuh1.y + 37), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
                flag_musuh = true;
            }
        }
    } 
    else
    {
        sudutMusuh1 = 0.0;
        flag_musuh = false;
    }

    //imshow("Teman", frameTeman);
    
    //cout << (int)sudutTeman1 << "|" << (int)sudutTeman2 << endl;
}




void R2CCitra::auto_segmentasi(Vec3b& selected_colour, bool& select_colour, int& g_segmentation_mode)
{
    Mat chosen_mask;
    Mat colour(1,1,CV_8UC3, selected_colour);
    Mat colour_HSV;
    cvtColor(colour, colour_HSV, COLOR_BGR2HSV);

    if(g_segmentation_mode == 0) //lapangan
    {
        if(select_colour == true)
        {
            int toleransi = 20;
            int h1 = colour_HSV.at<Vec3b>(0,0)[0];
            int s1 = colour_HSV.at<Vec3b>(0,0)[1];
            int v1 = colour_HSV.at<Vec3b>(0,0)[2];

            bb_hmn = h1 - toleransi;
            bb_smn = max(0, s1 - 50);
            bb_vmn = max(0, v1 - 50);
            bb_hmx = h1 + toleransi;
            bb_smx = min(255, s1 + 50);
            bb_vmx = min(255, v1 + 50);

            fstream data_pointer("kalibrasi/lapangan.txt");
            data_pointer << bb_hmn << "\n"
                         << bb_smn << "\n"
                         << bb_vmn << "\n"
                         << bb_hmx << "\n"
                         << bb_smx << "\n"
                         << bb_vmx;
            data_pointer.close();

            Scalar batasbawah(bb_hmn, bb_smn, bb_vmn);
            Scalar batasatas(bb_hmx, bb_smx, bb_vmx);
            inRange(colour_HSV, batasbawah, batasatas, chosen_mask);
        }
    }
    if(g_segmentation_mode == 1) //bola
    {
        if(select_colour == true)
        {
            int toleransi = 20;
            int h1 = colour_HSV.at<Vec3b>(0,0)[0];
            int s1 = colour_HSV.at<Vec3b>(0,0)[1];
            int v1 = colour_HSV.at<Vec3b>(0,0)[2];

            bb_hmn = h1 - toleransi;
            bb_smn = max(0, s1 - 50);
            bb_vmn = max(0, v1 - 50);
            bb_hmx = h1 + toleransi;
            bb_smx = min(255, s1 + 50);
            bb_vmx = min(255, v1 + 50);

            fstream data_pointer("kalibrasi/bola.txt");
            data_pointer << bb_hmn << "\n"
                         << bb_smn << "\n"
                         << bb_vmn << "\n"
                         << bb_hmx << "\n"
                         << bb_smx << "\n"
                         << bb_vmx;
            data_pointer.close();

            Scalar batasbawah(bb_hmn, bb_smn, bb_vmn);
            Scalar batasatas(bb_hmx, bb_smx, bb_vmx);
            inRange(colour_HSV, batasbawah, batasatas, chosen_mask);
        }
    }
    if(g_segmentation_mode == 2) //gawang depan
    {
        if(select_colour == true)
        {
            int toleransi = 20;
            int h1 = colour_HSV.at<Vec3b>(0,0)[0];
            int s1 = colour_HSV.at<Vec3b>(0,0)[1];
            int v1 = colour_HSV.at<Vec3b>(0,0)[2];

            bb_hmn = h1 - toleransi;
            bb_smn = max(0, s1 - 50);
            bb_vmn = max(0, v1 - 50);
            bb_hmx = h1 + toleransi;
            bb_smx = min(255, s1 + 50);
            bb_vmx = min(255, v1 + 50);

            fstream data_pointer("kalibrasi/gawang_depan.txt");
            data_pointer << bb_hmn << "\n"
                         << bb_smn << "\n"
                         << bb_vmn << "\n"
                         << bb_hmx << "\n"
                         << bb_smx << "\n"
                         << bb_vmx;
            data_pointer.close();

            Scalar batasbawah(bb_hmn, bb_smn, bb_vmn);
            Scalar batasatas(bb_hmx, bb_smx, bb_vmx);
            inRange(colour_HSV, batasbawah, batasatas, chosen_mask);
        }
    }
    if(g_segmentation_mode == 3) //teman depan
    {
        if(select_colour == true)
        {
            int toleransi = 20;
            int h1 = colour_HSV.at<Vec3b>(0,0)[0];
            int s1 = colour_HSV.at<Vec3b>(0,0)[1];
            int v1 = colour_HSV.at<Vec3b>(0,0)[2];

            bb_hmn = h1 - toleransi;
            bb_smn = max(0, s1 - 50);
            bb_vmn = max(0, v1 - 50);
            bb_hmx = h1 + toleransi;
            bb_smx = min(255, s1 + 50);
            bb_vmx = min(255, v1 + 50);

            fstream data_pointer("kalibrasi/teman_depan.txt");
            data_pointer << bb_hmn << "\n"
                         << bb_smn << "\n"
                         << bb_vmn << "\n"
                         << bb_hmx << "\n"
                         << bb_smx << "\n"
                         << bb_vmx;
            data_pointer.close();

            Scalar batasbawah(bb_hmn, bb_smn, bb_vmn);
            Scalar batasatas(bb_hmx, bb_smx, bb_vmx);
            inRange(colour_HSV, batasbawah, batasatas, chosen_mask);
        }
    }
    if(g_segmentation_mode == 4) //musuh depan
    {
        if(select_colour == true)
        {
            int toleransi = 20;
            int h1 = colour_HSV.at<Vec3b>(0,0)[0];
            int s1 = colour_HSV.at<Vec3b>(0,0)[1];
            int v1 = colour_HSV.at<Vec3b>(0,0)[2];

            bb_hmn = h1 - toleransi;
            bb_smn = max(0, s1 - 50);
            bb_vmn = max(0, v1 - 50);
            bb_hmx = h1 + toleransi;
            bb_smx = min(255, s1 + 50);
            bb_vmx = min(255, v1 + 50);

            fstream data_pointer("kalibrasi/musuh_depan.txt");
            data_pointer << bb_hmn << "\n"
                         << bb_smn << "\n"
                         << bb_vmn << "\n"
                         << bb_hmx << "\n"
                         << bb_smx << "\n"
                         << bb_vmx;
            data_pointer.close();

            Scalar batasbawah(bb_hmn, bb_smn, bb_vmn);
            Scalar batasatas(bb_hmx, bb_smx, bb_vmx);
            inRange(colour_HSV, batasbawah, batasatas, chosen_mask);
        }
    }
    if(g_segmentation_mode == 5) //musuh omni
    {
        if(select_colour == true)
        {
            int toleransi = 20;
            int h1 = colour_HSV.at<Vec3b>(0,0)[0];
            int s1 = colour_HSV.at<Vec3b>(0,0)[1];
            int v1 = colour_HSV.at<Vec3b>(0,0)[2];

            bb_hmn = h1 - toleransi;
            bb_smn = max(0, s1 - 50);
            bb_vmn = max(0, v1 - 50);
            bb_hmx = h1 + toleransi;
            bb_smx = min(255, s1 + 50);
            bb_vmx = min(255, v1 + 50);

            fstream data_pointer("kalibrasi/musuh.txt");
            data_pointer << bb_hmn << "\n"
                         << bb_smn << "\n"
                         << bb_vmn << "\n"
                         << bb_hmx << "\n"
                         << bb_smx << "\n"
                         << bb_vmx;
            data_pointer.close();

            Scalar batasbawah(bb_hmn, bb_smn, bb_vmn);
            Scalar batasatas(bb_hmx, bb_smx, bb_vmx);
            inRange(colour_HSV, batasbawah, batasatas, chosen_mask);
        }
    }
    if(g_segmentation_mode == 6) //teman omni
    {
        if(select_colour == true)
        {
            int toleransi = 20;
            int h1 = colour_HSV.at<Vec3b>(0,0)[0];
            int s1 = colour_HSV.at<Vec3b>(0,0)[1];
            int v1 = colour_HSV.at<Vec3b>(0,0)[2];

            bb_hmn = h1 - toleransi;
            bb_smn = max(0, s1 - 50);
            bb_vmn = max(0, v1 - 50);
            bb_hmx = h1 + toleransi;
            bb_smx = min(255, s1 + 50);
            bb_vmx = min(255, v1 + 50);

            fstream data_pointer("kalibrasi/teman.txt");
            data_pointer << bb_hmn << "\n"
                         << bb_smn << "\n"
                         << bb_vmn << "\n"
                         << bb_hmx << "\n"
                         << bb_smx << "\n"
                         << bb_vmx;
            data_pointer.close();

            Scalar batasbawah(bb_hmn, bb_smn, bb_vmn);
            Scalar batasatas(bb_hmx, bb_smx, bb_vmx);
            inRange(colour_HSV, batasbawah, batasatas, chosen_mask);
        }
    }
}

int search_line_extend = 0;
int add_offset = 0;

void R2CCitra::deteksi_objek_hitam(Mat& frame, int& nLines, vector<int>& jarak_obs, vector<float>& angle_obs, float& avg_angle, float& sum_angle, float& num_angle, STM32Data& mSTM32Data)
{
    float line_angles;
    Point center(frame.cols/2, frame.rows/2);

    Vec3b l_tresh(55,45,15);
    Vec3b h_tresh(70,60,30);
     
    if(mSTM32Data.GRIDY == 20)
    {
        add_offset = 0;
        search_line_extend = 45;
    }
    else if(mSTM32Data.GRIDY == 19)
    {
        add_offset = 0;
        search_line_extend = 75;   
    }
    else if(mSTM32Data.GRIDY == 18)
    {
        add_offset = 0;
        search_line_extend = 70;   
    }
    else if(mSTM32Data.GRIDY <= 17)
    {
        add_offset = 40;
        search_line_extend = 90;   
    }


    for (int i = 0; i < nLines; i++)
    {
        Point start
        (
            0 + i * 10,
            frame.rows
        );
        Point end
        (
            0 + i * 10,
            frame.rows - search_line_extend
        );

        LineIterator it(frame, start, end, 8);
        bool found = false;
        for (int j = 0; j < it.count; j++, ++it)
        { 
            Vec3b pixel = frame.at<Vec3b>(it.pos());
            if((pixel[0] >= l_tresh[0]) && (pixel[0] <= h_tresh[0]) &&
               (pixel[1] >= l_tresh[1]) && (pixel[1] <= h_tresh[1]) &&
               (pixel[2] >= l_tresh[2]) && (pixel[2] <= h_tresh[2]))
            {
                if((it.pos().x > gawang_kanan.x) && (it.pos().x < gawang_kiri.x))
                {
                    circle(frame, it.pos(), 3, Scalar(240, 32, 160), -1);
                    jarak_obs[i] = it.pos().x + add_offset;
                    found = true;
                    break;
                }
            }
            if (found == false)
            {
                jarak_obs[i] = -1;
                angle_obs[i] = -1;
            }
        }
        line(frame, start, end, Scalar(255, 255, 255), 1);
    } 
    for(int i = 0; i < nLines; i++)
    {
        if(jarak_obs[i] != -1)
        {
            sum_angle += jarak_obs[i];
            num_angle++;
        }
    }
    avg_angle = sum_angle/num_angle;
}

void R2CCitra::gambar_bola_dan_heading_robot(Mat& map, float& e_distance, float& ball_angle, Point& posisi_robot, Point& pos_bola, STM32Data& mSTM32Data, pcData& mPcData)
{
    float real_ball_distance = exp((e_distance + 54.37) / 33.03);
    float ball_angle_rads = ball_angle * CV_PI / 180;
    float robot_angle_rads = mSTM32Data.KOMPAS * CV_PI / 180;
    float global_angle = ball_angle_rads + robot_angle_rads - 1.57;
    
    pos_bola = Point
    (
        posisi_robot.x + (real_ball_distance * cos(global_angle)),
        posisi_robot.y + (real_ball_distance * sin(global_angle))
    );

    if(e_distance != 0)
    {
        circle(map, pos_bola, 5, Scalar(0,165,255), -1);
    }

    Point heading_robot
    (
        posisi_robot.x + (25 * cos(robot_angle_rads - 1.57)),
        posisi_robot.y + (25 * sin(robot_angle_rads - 1.57))
    );

    circle(map, posisi_robot, 25, Scalar(228,27,235), -1);
    line(map, posisi_robot, heading_robot, Scalar(0,0,0), 1);
}

int line_extend = 0;
void R2CCitra::ball_predict(Mat& frame, pcData& mPcData, STM32Data& mSTM32Data, Point& pos_bola, Point& center, int& catch_heading_deg, Point& predict_bola, Point& b_avged)
{
    b_delta = pos_bola - b_previous;

    if(b_delta.y >= 0.3)
    {
        b_heading = atan2(b_delta.y, b_delta.x); //(int)((b_head *180 / CV_PI) + 270) % 360 - 180;
        line_extend = 50;
    }
    else
    {
        line_extend = 0;
        b_heading = 1.57;
    }
    predict_bola = Point
    (
        pos_bola.x + (line_extend * cos(b_heading)),
        pos_bola.y + (line_extend * sin(b_heading))
    );

    save_position.push_back(predict_bola);
    if(save_position.size() > 10)
    {
        save_position.erase(save_position.begin());
    }

    for(auto pos : save_position)
    {
        if(b_delta.y >= 0.3)
        {
            sumX += pos.x;
            sumY += pos.y;
        }
        else
        {
            sumX += pos_bola.x;
            sumY += pos_bola.y;
        }
    }

    avgX = sumX / 10;
    avgY = sumY / 10;

    b_avged = Point(avgX, avgY);

    float catch_heading = atan2(b_avged.y - center.y, b_avged.x - center.x);
    catch_heading_deg = (int)((atan2(b_avged.y - center.y, b_avged.x - center.x) *180 / CV_PI) + 270) % 360 - 180;
    Point2f catch_dir
    (
        center.x + (30 * cos(catch_heading)),
        center.y + (30 * sin(catch_heading))
    );
    //line(frame, center, catch_dir, Scalar(255,0,0), 2);
    line(frame, pos_bola, b_avged, Scalar(255,255,0), 2);
    sumX = 0;
    sumY = 0;
    b_previous = pos_bola;
}



/*===================INISIALISASI================*/
bool R2CCitra::init()//int &indexcambawah, int &indexcamomni,float &arahGawangLawan)
{
	printf("R2CCitra: init run\n");
	R2CCitra::read_data();//indexcambawah,indexcamomni,arahGawangLawan);
	return true;
}
bool deteksi_center_target(Mat& frame_asli, Mat& frame_teman_depan, int& angle_bola, int& jarak_1, bool& kondisi){

}
