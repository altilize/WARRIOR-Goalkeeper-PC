#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "string.h"
#include "string"
#include <vector>
#include <map>
#include <mutex>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "library/TypeData.h"
#include "library/Camera.h"
#include "library/Citra.h"
#include "library/StrikerCamera.h"


#include <gtkmm.h>
#include "gui/include/SegmentationUi.hpp"
#include "library/StrikerCamera.h"

#define IMAGE_SIZE_WIDTH 640
#define IMAGE_SIZE_HEIGHT 480

#define SEGMENTED_IMAGE_SIZE_WIDTH 320
#define SEGMENTED_IMAGE_SIZE_HEIGHT 240

using namespace std;
using namespace cv;

/**
 * mode source
 * 0 = dari file
 * 1 = dari camera
 */
int g_source_mode = 0;
int g_image_index = 0;
int g_segmentation_mode = 0;
// int jarak;

STM32Data mSTM32Data;

vector<Camera> g_camera_list;
Camera selectedCamera;
int g_app_run = 1;

Mat frameAsli, frameSegmentasi, frameRGB, frameHSV, frameLAB;
// Mat frameHSV2RGB;//(Size(640,480), CV_16UC3);
VideoCapture cap;
bool status = false;
float tes_sudut, tes_jarak;
int tes_x, tes_y;
std::mutex imageMutex;
Glib::Dispatcher dispatcher;
Gtk::ComboBoxText *comboBoxCamera;
Gtk::Button *imagePrev, *imageNext, *imageSave;
Gtk::Scale *scaleHMin, *scaleHMax, *scaleSMin, *scaleSMax, *scaleVMin, *scaleVMax;
Gtk::Label *labelInfo;

int hMin[7] = {0, 0, 0, 0, 0, 0, 0};
int hMax[7] = {255, 255, 255, 255, 255, 255, 255};
int sMin[7] = {0, 0, 0, 0, 0, 0, 0};
int sMax[7] = {255, 255, 255, 255, 255, 255, 255};
int vMin[7] = {0, 0, 0, 0, 0, 0, 0};
int vMax[7] = {255, 255, 255, 255, 255, 255, 255};

// vector<Vec2i> hor;
// vector<Vec2i> hor1;

Vec3b selected_colour;
bool select_colour = false;

void mouseCallback(int event, int x, int y, int flags, void*userdata)
{
  if(event == EVENT_LBUTTONDOWN)
  {
    Mat*image = reinterpret_cast<Mat*>(userdata);
    selected_colour = image->at<Vec3b>(Point(x, y));
    select_colour = true;
  }
}


void updateUI()
{
  string info;
  if (g_source_mode == 1)
  {
    info += "Mode: Camera\n";
    imageSave->set_sensitive(true);
    imagePrev->set_sensitive(false);
    imageNext->set_sensitive(false);
  }
  else if (g_source_mode == 0)
  {
    info += "Mode: File\n";
    imageSave->set_sensitive(false);
    imagePrev->set_sensitive(true);
    imageNext->set_sensitive(true);
    info += "Index gambar: " + std::to_string(g_image_index);
    info += "\n";
  }

  if (g_segmentation_mode == 0)
  {
    info += "Kalibrasi: Lapangan\n";
  }
  else if (g_segmentation_mode == 1)
  {
    info += "Kalibrasi: Bola\n";
  }
  else if (g_segmentation_mode == 2)
  {
    info += "Kalibrasi: Gawang Depan\n";
  }
  else if (g_segmentation_mode == 3)
  {
    info += "Kalibrasi: Teman Depan\n";
  }
  else if (g_segmentation_mode == 4)
  {
    info += "Kalibrasi: Musuh Depan\n";
  }
  else if (g_segmentation_mode == 5)
  {
    info += "Kalibrasi: Musuh\n";
  }
  else if (g_segmentation_mode == 6)
  {
    info += "Kalibrasi: Teman\n";
  }

  labelInfo->set_label(info);
}

void setSourceMode(int mode)
{
  if (g_source_mode == 0 && mode == 1)
  {
    // printf("Mode camera\n");
    g_source_mode = mode;
    comboBoxCamera->set_active_text("");
    comboBoxCamera->set_sensitive(true);

    updateUI();
  }
  else if (g_source_mode == 1 && mode == 0)
  {
    // printf("Mode gambar\n");
    g_source_mode = mode;
    g_image_index = 0;
    comboBoxCamera->set_active_text("");
    comboBoxCamera->set_sensitive(false);

    updateUI();

    if (cap.isOpened())
    {
      imageMutex.lock();
      cap.release();
      imageMutex.unlock();
    }
  }
}

void loadParameter()
{
  for (int i = 0; i <= 6; i++)
  {
    string path = "kalibrasi/";
    if (i == 0)
    {
      path += "lapangan.txt";
    }
    else if (i == 1)
    {
      path += "bola.txt";
    }
    else if (i == 2)
    {
      path += "gawang_depan.txt";
    }
    else if (i == 3)
    {
      path += "teman_depan.txt";
    }
    else if (i == 4)
    {
      path += "musuh_depan.txt";
    }
    else if (i == 5)
    {
      path += "musuh.txt";
    }
    else if (i == 6)
    {
      path += "teman.txt";
    }
    // else if(segmentationMode == 5){
    //     info += "Kalibrasi: Teman\n";
    // }

    FILE *input = fopen(path.c_str(), "r");
    if (input == NULL)
    {
      printf("Error open file: %s\n", path.c_str());
      return;
    }

    fscanf(input, "%i\n%i\n%i\n%i\n%i\n%i\n", &hMin[i], &sMin[i], &vMin[i], &hMax[i], &sMax[i], &vMax[i]);
    fclose(input);
  }
}

void setSegmentationMode(int mode)
{
  if (g_segmentation_mode != mode)
  {
    g_segmentation_mode = mode;

    loadParameter();

    scaleHMin->set_value(hMin[g_segmentation_mode]);
    scaleHMax->set_value(hMax[g_segmentation_mode]);

    scaleSMin->set_value(sMin[g_segmentation_mode]);
    scaleSMax->set_value(sMax[g_segmentation_mode]);

    scaleVMin->set_value(vMin[g_segmentation_mode]);
    scaleVMax->set_value(vMax[g_segmentation_mode]);

    updateUI();
  }
}

void onCameraChanged()
{
  if (comboBoxCamera->get_active_text().size() > 0)
  {
    int cameraIndex = 0;
    char temp[100];
    sscanf(comboBoxCamera->get_active_text().c_str(), "%i#%s", &cameraIndex, temp);

    selectedCamera = g_camera_list[cameraIndex];
    if (cap.isOpened())
    {
      imageMutex.lock();
      cap.release();
      imageMutex.unlock();
    }
    cap.open(selectedCamera.index, cv::CAP_V4L2);
    cap.set(CAP_PROP_FRAME_WIDTH, IMAGE_SIZE_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, IMAGE_SIZE_HEIGHT);

    R2CCamera::setParameterCamera(selectedCamera);
  }
}

void prevImage()
{
  if (g_image_index > 0)
  {
    g_image_index--;
    updateUI();
  }
}

void nextImage()
{
  g_image_index++;
  updateUI();
}

int file_counter(const char *path)
{
  FILE *fp;
  char buffer[1024];

  string command = "/bin/ls ";
  command += path;
  command += " | /usr/bin/wc -l";

  fp = popen(command.c_str(), "r");
  if (fp == NULL)
  {
    printf("failed to run command\n");
    return 0;
  }

  while (fgets(buffer, sizeof(buffer) - 1, fp) != NULL)
  {
  }
  pclose(fp);
  int index = atoi(buffer);
  return index;
}

void saveImage()
{
  system("mkdir -p data");

  int image_count = file_counter("data");
  string savingPict = "data/" + to_string(image_count) + ".jpg";

  imwrite(savingPict, frameAsli);

  printf("Gambar garis tersimpan\n");
}

// cv::Mat frameLapangan, frameBola, frameGawang, frameGaris, frameTemp;
cv::Mat frameLapangan, frameBola, frameGawangDepan, frameGawangOmni, frameMusuhDepan, frameMusuh, frameTeman, frameTemanDepan, frameTemp;
int koordsX, koordsY;
float test_jarak;
float test_sudut;

Point2f t_koordinat_teman1;
Point2f t_koordinat_teman2;
int t_angle_temen1, t_angle_temen2;
bool temen1 = false;
int t_jarak_temen1, t_jarak_temen2;

Point2f t_koordinat_musuh1;
Point2f t_koordinat_musuh2;
Point2f t_koordinat_musuh3;
float t_angle_musuh1, t_angle_musuh2, t_angle_musuh3;

int test_goalWidth, test_goalAngle;
float test_goalDistance;
int test_left, test_right;
bool is_goal_visible = false;
bool is_post_visible = false;
bool flagGawang;
int sudut_gawang;

extern vector<Vec2i> hor;
void segmentationColor()
{

  frameSegmentasi = frameAsli.clone();
  GaussianBlur(frameSegmentasi, frameSegmentasi, Size(1, 1), 0);
  cv::cvtColor(frameSegmentasi, frameTemp, COLOR_BGR2HSV);

  cv::inRange(frameTemp, cv::Scalar(hMin[0], sMin[0], vMin[0]), cv::Scalar(hMax[0], sMax[0], vMax[0]), frameLapangan);
  cv::inRange(frameTemp, cv::Scalar(hMin[1], sMin[1], vMin[1]), cv::Scalar(hMax[1], sMax[1], vMax[1]), frameBola);
  cv::inRange(frameTemp, cv::Scalar(hMin[2], sMin[2], vMin[2]), cv::Scalar(hMax[2], sMax[2], vMax[2]), frameGawangDepan);
  cv::inRange(frameTemp, cv::Scalar(hMin[3], sMin[3], vMin[3]), cv::Scalar(hMax[3], sMax[3], vMax[3]), frameTemanDepan);
  cv::inRange(frameTemp, cv::Scalar(hMin[4], sMin[4], vMin[4]), cv::Scalar(hMax[4], sMax[4], vMax[4]), frameMusuhDepan);
  cv::inRange(frameTemp, cv::Scalar(hMin[5], sMin[5], vMin[5]), cv::Scalar(hMax[5], sMax[5], vMax[5]), frameMusuh);
  cv::inRange(frameTemp, cv::Scalar(hMin[6], sMin[6], vMin[6]), cv::Scalar(hMax[6], sMax[6], vMax[6]), frameTeman);
}

void segmented_frame_from_mode(cv::Mat &source, int mode)
{
  cv::inRange(source,
              cv::Scalar(hMin[mode], sMin[mode], vMin[mode]),
              cv::Scalar(hMax[mode], sMax[mode], vMax[mode]),
              source);
}

void captureImage()
{
  Mat frame_omni_;
  while (g_app_run)
  {
    imageMutex.lock();
    if (g_source_mode == 1 && cap.isOpened())
    {
      cap.read(frameAsli);

      if (!frameAsli.data)
      {
        printf("Tidak dapat mengambil gambar\n");
        imageMutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }
    }
    else if (g_source_mode == 0)
    {
      string framePath = "data/";
      framePath += std::to_string(g_image_index) + ".jpg";
      frameAsli = imread(framePath);

      if (!frameAsli.data)
      {
        printf("Tidak dapat mengambil gambar dari %s\n", framePath.c_str());
        imageMutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }
    }

    //GaussianBlur(frameAsli, frameAsli, Size(1, 1), 0);
    //cv::medianBlur(frameAsli, frameAsli, 3);
    cv::cvtColor(frameAsli, frameHSV, COLOR_BGR2HSV);

    cv::Mat frame_hsv_field = frameHSV.clone();
    cv::Mat frame_hsv_ball = frameHSV.clone();
    cv::Mat frame_hsv_friend = frameHSV.clone();
    cv::Mat frame_hsv_enemy = frameHSV.clone();
    cv::Mat frame_hsv_goalPostFront = frameHSV.clone();
    cv::Mat frame_hsv_friendFront = frameHSV.clone();
    cv::Mat frame_hsv_enemyFront = frameHSV.clone();
    segmented_frame_from_mode(frame_hsv_field, 0);
    segmented_frame_from_mode(frame_hsv_ball, 1);
    segmented_frame_from_mode(frame_hsv_goalPostFront, 2);
    segmented_frame_from_mode(frame_hsv_friendFront, 3);
    segmented_frame_from_mode(frame_hsv_enemyFront, 4);
    segmented_frame_from_mode(frame_hsv_enemy, 5);
    segmented_frame_from_mode(frame_hsv_friend, 6);

    setMouseCallback("Autosegmentasi", mouseCallback, &frameAsli);
    R2CCitra::auto_segmentasi(selected_colour, select_colour, g_segmentation_mode);
    imshow("Autosegmentasi", frameAsli);



    cv::inRange(frameHSV,
                cv::Scalar(hMin[g_segmentation_mode], sMin[g_segmentation_mode], vMin[g_segmentation_mode]),
                cv::Scalar(hMax[g_segmentation_mode], sMax[g_segmentation_mode], vMax[g_segmentation_mode]),
                frameHSV);

    R2CCitra::convex_hull(frameAsli, frame_hsv_field);
    //    } else if(g_segmentation_mode == 1) {
    bool is_ball_visible = false;
    float angle = 0, e_distance = 0;
    int coordinate_x = 0, coordinate_y = 0;
    bool goal_flag = false;
    int goal_angle = 0;
    bool is_friend_visible = false;
    int t_angleFrontFriend1 = 0;
    int t_angleFrontFriend2 = 0;
    bool t_flagFrontFriend1 = false;
    bool t_flagFrontFriend2 = false;
    bool is_enemy_visible = false;
    int t_angleFrontEnemy1 = 0;
    int t_angleFrontEnemy2 = 0;
    bool t_flagFrontEnemy1 = false;
    bool t_flagFrontEnemy2 = false;

    bool t_param_gawang = false;
    int t_goalAngle = 0;
    float dummyPosition;

    cv::Point t_centerEnemy;

    R2CCitra::deteksi_bola_omni(is_ball_visible, frameAsli, frame_hsv_ball, angle, e_distance, coordinate_x, coordinate_y);
    R2CCitra::deteksi_teman_omni(frameAsli, frame_hsv_friend, t_koordinat_teman1, t_angle_temen1, t_jarak_temen1, temen1);

    R2CCitra::deteksi_musuh_depan(frameAsli, frame_hsv_enemyFront, t_angleFrontEnemy1, t_angleFrontEnemy2, t_flagFrontEnemy1, t_flagFrontEnemy2, t_centerEnemy);
    R2CCitra::deteksi_gawang_depan(frameAsli, frame_hsv_goalPostFront, t_param_gawang, t_goalAngle, t_angleFrontEnemy1, t_centerEnemy, mSTM32Data, dummyPosition);


    cv::cvtColor(frameHSV, frameHSV, COLOR_GRAY2RGB);
    cv::cvtColor(frameAsli, frameRGB, COLOR_BGR2RGB);

    imageMutex.unlock();
    dispatcher.emit();

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
}

int main(int argc, char *argv[])
{
  R2CCamera::loadCameras(g_camera_list);
  printf("Camera size: %li\n\n", g_camera_list.size());

  auto app = Gtk::Application::create(argc, argv, "r2c.warrior.segmentation");

  // cap.open(0);//selectedCamera.index);
  cap.set(CAP_PROP_FRAME_WIDTH, IMAGE_SIZE_WIDTH);
  cap.set(CAP_PROP_FRAME_HEIGHT, IMAGE_SIZE_HEIGHT);

  Glib::RefPtr<Gtk::Builder> builder = Gtk::Builder::create_from_file("gui/ui/segmentation_ui.glade");

  SegmentationUi *segmentation_calibration;
  builder->get_widget_derived("SegmentationWindow", segmentation_calibration);

  updateUI();
  reloadParameter();

  dispatcher.connect([&]()
                     {
    imageMutex.lock();

    cv::Mat resized_rgb, resized_hsv;
    cv::resize(frameRGB, resized_rgb, Size(IMAGE_SIZE_WIDTH, IMAGE_SIZE_HEIGHT));
    cv::resize(frameHSV, resized_hsv, Size(SEGMENTED_IMAGE_SIZE_WIDTH, SEGMENTED_IMAGE_SIZE_HEIGHT));

    segmentation_calibration->updateImage(resized_rgb, resized_hsv);
    imageMutex.unlock(); });

  std::thread cameraThread = std::thread(&captureImage);
  app->run(*segmentation_calibration);
  cameraThread.join();

  return 0;
}

void turnOffApp()
{
  g_app_run = 0;
}

void resetParameter()
{
  hMin[g_segmentation_mode] = 0;
  hMax[g_segmentation_mode] = 255;
  sMin[g_segmentation_mode] = 0;
  sMax[g_segmentation_mode] = 255;
  vMin[g_segmentation_mode] = 0;
  vMax[g_segmentation_mode] = 255;

  scaleHMin->set_value(hMin[g_segmentation_mode]);
  scaleHMax->set_value(hMax[g_segmentation_mode]);

  scaleSMin->set_value(sMin[g_segmentation_mode]);
  scaleSMax->set_value(sMax[g_segmentation_mode]);

  scaleVMin->set_value(vMin[g_segmentation_mode]);
  scaleVMax->set_value(vMax[g_segmentation_mode]);
}

void reloadParameter()
{
  loadParameter();

  scaleHMin->set_value(hMin[g_segmentation_mode]);
  scaleHMax->set_value(hMax[g_segmentation_mode]);

  scaleSMin->set_value(sMin[g_segmentation_mode]);
  scaleSMax->set_value(sMax[g_segmentation_mode]);

  scaleVMin->set_value(vMin[g_segmentation_mode]);
  scaleVMax->set_value(vMax[g_segmentation_mode]);
}

void saveParameter()
{
  string path = "kalibrasi/";
  if (g_segmentation_mode == 0)
  {
    path += "lapangan.txt";
  }
  else if (g_segmentation_mode == 1)
  {
    path += "bola.txt";
  }
  else if (g_segmentation_mode == 2)
  {
    path += "gawang_depan.txt";
  }
  else if (g_segmentation_mode == 3)
  {
    path += "teman_depan.txt";
  }
  else if (g_segmentation_mode == 4)
  {
    path += "musuh_depan.txt";
  }
  else if (g_segmentation_mode == 5)
  {
    path += "musuh.txt";
  }
  else if (g_segmentation_mode == 6)
  {
    path += "teman.txt";
  }

  system("mkdir -p kalibrasi");

  FILE *out = fopen(path.c_str(), "w");

  fprintf(out, "%i\n", hMin[g_segmentation_mode]);
  fprintf(out, "%i\n", sMin[g_segmentation_mode]);
  fprintf(out, "%i\n", vMin[g_segmentation_mode]);
  fprintf(out, "%i\n", hMax[g_segmentation_mode]);
  fprintf(out, "%i\n", sMax[g_segmentation_mode]);
  fprintf(out, "%i\n", vMax[g_segmentation_mode]);
  fclose(out);

  Gtk::MessageDialog *dialog = new Gtk::MessageDialog("Parameter saved", false, Gtk::MESSAGE_INFO, Gtk::BUTTONS_CLOSE);
  int result = dialog->run();
  if (result == Gtk::RESPONSE_CLOSE)
  {
    dialog->close();
  }
}