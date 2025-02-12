#ifndef SELECT_CAMERA_H
#define SELECT_CAMERA_H

#include <gtkmm.h>
#include <vector>
#include "../library/TypeData.h"

class SelectCamera: public Gtk::Window{
    public:
        SelectCamera();
        virtual ~SelectCamera();

    protected:
        Gtk::Box cameraContainer;
};

extern vector<Camera> cameraList;
extern void selectCamera(Camera cam, Gtk::Window* window);

#endif