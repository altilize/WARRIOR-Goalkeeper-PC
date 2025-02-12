#include <gtkmm.h>
#include "../library/TypeData.h"
#include <string>
#include <iostream>

#include "KalibrasiKamera.h"

class ParameterButton : public Gtk::Frame{
    public:
        ParameterButton(Parameter param);
        virtual ~ParameterButton();

    protected:
        Gtk::Box buttonList;
};

extern void setParameter(string name, int value);

ParameterButton::ParameterButton(Parameter param): buttonList(Gtk::ORIENTATION_HORIZONTAL, 5){
    this->set_label(param.name);

    this->add(buttonList);

    Gtk::Button* buttonMinus10 = new Gtk::Button("-10");
    buttonMinus10->signal_clicked()
        .connect(sigc::bind<string, int>(sigc::ptr_fun(setParameter), param.name, -10));
    buttonList.pack_start(*buttonMinus10);

    Gtk::Button* buttonMinus1 = new Gtk::Button("-1");
    buttonMinus1->signal_clicked()
        .connect(sigc::bind<string, int>(sigc::ptr_fun(setParameter), param.name, -1));
    buttonList.pack_start(*buttonMinus1);

    Gtk::Button* buttonPlus1 = new Gtk::Button("+1");
    buttonPlus1->signal_clicked()
        .connect(sigc::bind<string, int>(sigc::ptr_fun(setParameter), param.name, 1));
    buttonList.pack_start(*buttonPlus1);

    Gtk::Button* buttonPlus10 = new Gtk::Button("+10");
    buttonPlus10->signal_clicked()
        .connect(sigc::bind<string, int>(sigc::ptr_fun(setParameter), param.name, 10));
    buttonList.pack_start(*buttonPlus10);

    this->show_all_children();
}

ParameterButton::~ParameterButton(){

}

CameraCallibration::CameraCallibration(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade)
:Gtk::Window(cobject), builder(refGlade){
    refGlade->get_widget("ParameterButtonList", parameterButtonList);
    refGlade->get_widget("ParameterLabel", parameterLabel);
    refGlade->get_widget("CameraFrame", imageBox);

    refGlade->get_widget("ButtonReload", buttonReload);
    refGlade->get_widget("ButtonReset", buttonReset);
    refGlade->get_widget("ButtonSave", buttonSave);

    buttonReload->signal_clicked().connect(sigc::ptr_fun(reloadParameter));
    buttonReset->signal_clicked().connect(sigc::ptr_fun(resetParameter));
    buttonSave->signal_clicked().connect(sigc::ptr_fun(saveParameter));

    this->signal_hide().connect(sigc::ptr_fun(turnOffApp));
}

CameraCallibration::~CameraCallibration(){

}

void CameraCallibration::updateImage(cv::Mat & frame) {
	if(!frame.empty()) {
		this->imageBox->set(Gdk::Pixbuf::create_from_data(frame.data, Gdk::COLORSPACE_RGB, false, 8, frame.cols, frame.rows, frame.step));
		this->imageBox->queue_draw();
	}
}

void CameraCallibration::setCamera(Camera cam){
    set_title("Callibration Camera - "+cam.name);

    for(auto params: cam.parameters){
        Parameter param = params.second;

        ParameterButton* button = new ParameterButton(param);
        parameterButtonList->pack_start(*button);
    }

    updateInfo();
    this->show_all_children();
}

