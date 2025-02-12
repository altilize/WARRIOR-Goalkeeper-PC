#ifndef SEGMENTATION_CALLIBRATION_H
#define SEGMENTATION_CALLIBRATION_H

#include <gtkmm.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "../library/TypeData.h"
#include <string>

class SegmentationCallibration : public Gtk::Window{
    public:
        public:
        SegmentationCallibration(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade);
        virtual ~SegmentationCallibration();
        void updateImage(cv::Mat &frameRGB, cv::Mat &frameHSV);

    private:
        Glib::RefPtr<Gtk::Builder> builder;

    protected:
        Gtk::Box* parameterButtonList;
        Gtk::Image *imageRGB, *imageHSV; 

        Gtk::RadioButton *radioSourceFile, *radioSourceCamera;
        Gtk::RadioButton *radioTypeLapangan, *radioTypeBola, *radioTypeGawang, *radioTypeGaris,*radioTypeMusuh, *radioTypeTeman;

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
extern vector<Camera> cameraList;
extern Gtk::Button *imagePrev, *imageNext, *imageSave;
extern Gtk::Label *labelInfo;
extern Gtk::Scale *scaleHMin, *scaleHMax, *scaleSMin, *scaleSMax, *scaleVMin, *scaleVMax;

extern int segmentationMode;
extern int sMin[6];
extern int sMax[6];
extern int hMin[6];
extern int hMax[6];
extern int vMin[6];
extern int vMax[6];

#endif