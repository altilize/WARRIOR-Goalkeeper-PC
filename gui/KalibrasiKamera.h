#ifndef CAMERA_CALLIBRATION_H
#define CAMERA_CALLIBRATION_H

#include <gtkmm.h>
#include "../library/TypeData.h"
#include <opencv2/opencv.hpp>

class CameraCallibration : public Gtk::Window{
    public:
        CameraCallibration(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade);
        virtual ~CameraCallibration();
        void setCamera(Camera);
        void updateImage(cv::Mat & frame);

    private:
        Glib::RefPtr<Gtk::Builder> builder;

    protected:
        Gtk::Box* parameterButtonList;
        Gtk::Image* imageBox; 
        Gtk::Button* buttonReset, *buttonReload, *buttonSave;
};

extern Gtk::Label* parameterLabel;
extern void updateInfo();
extern void resetParameter();
extern void reloadParameter();
extern void saveParameter();
extern void turnOffApp();

#endif