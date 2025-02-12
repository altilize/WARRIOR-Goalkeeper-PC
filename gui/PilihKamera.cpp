#include "PilihKamera.h"
#include <iostream>

SelectCamera::SelectCamera() : cameraContainer(Gtk::ORIENTATION_VERTICAL, 10){
    set_title("Select camera");
    set_border_width(10);
    add(cameraContainer);

    for(int i=0;i<cameraList.size();i++){
        Camera cam = cameraList[i];
        Gtk::Button* button = new Gtk::Button(cam.name+" : "+std::to_string(cam.index));
        button->signal_clicked().connect(sigc::bind<Camera, Gtk::Window*>(sigc::ptr_fun(selectCamera), cam, this));
        cameraContainer.pack_start(*button);
    }
    show_all_children();
}

SelectCamera::~SelectCamera(){

}