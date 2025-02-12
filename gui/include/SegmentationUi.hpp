#ifndef SEGMENTATION_CALIBRATION_H
#define SEGMENTATION_CALIBRATION_H

#include <gtkmm.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "../../library/TypeData.h"
#include <string>

class SegmentationUi : public Gtk::Window {
 public:
  SegmentationUi(BaseObjectType *c_object, const Glib::RefPtr<Gtk::Builder> &ref_glade);
  virtual ~SegmentationUi();
  void updateImage(cv::Mat &frameRGB, cv::Mat &frameHSV);

 private:
  Glib::RefPtr<Gtk::Builder> builder;

 protected:
  Gtk::Box *parameterButtonList;
  Gtk::Image *imageRGB, *imageHSV;

  Gtk::RadioButton *radioSourceFile, *radioSourceCamera;
  Gtk::RadioButton *radioTypeLapangan, *radioTypeBola, *radioTypeGawangDepan, *radioTypeTemanDepan, *radioTypeMusuhDepan, *radioTypeMusuh, *radioTypeTeman;

  Gtk::Button *buttonResetParameter, *buttonReloadParameter, *buttonSaveParameter;
};

extern void turnOffApp();
extern void setSourceMode(int mode);
extern void setSegmentationMode(int mode);
extern void onCameraChanged();

extern void prevImage();
extern void nextImage();
extern void saveImage();

extern void resetParameter();
extern void reloadParameter();
extern void saveParameter();

extern Gtk::ComboBoxText *comboBoxCamera;
extern vector<Camera> g_camera_list;
extern Gtk::Button *imagePrev, *imageNext, *imageSave;
extern Gtk::Label *labelInfo;
extern Gtk::Scale *scaleHMin, *scaleHMax, *scaleSMin, *scaleSMax, *scaleVMin, *scaleVMax;

extern int g_segmentation_mode;
extern int hMin[7];
extern int hMax[7];
extern int sMin[7];
extern int sMax[7];
extern int vMin[7];
extern int vMax[7];

#endif