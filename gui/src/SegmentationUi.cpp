#include "../include/SegmentationUi.hpp"

#include <iostream>

SegmentationUi::SegmentationUi(BaseObjectType* c_object, const Glib::RefPtr<Gtk::Builder>& ref_glade)
:Gtk::Window(c_object), builder(ref_glade){
    ref_glade->get_widget("imageRGB", imageRGB);
    ref_glade->get_widget("imageHSV", imageHSV);

	ref_glade->get_widget("radioSourceFile", radioSourceFile);
	ref_glade->get_widget("radioSourceCamera", radioSourceCamera);

	radioSourceFile->signal_toggled().connect(sigc::bind<int>(sigc::ptr_fun(setSourceMode), 0));
	radioSourceCamera->signal_toggled().connect(sigc::bind<int>(sigc::ptr_fun(setSourceMode), 1));

	ref_glade->get_widget("comboBoxCamera", comboBoxCamera);
	for(int i=0;i<g_camera_list.size();i++){
		comboBoxCamera->append(std::to_string(i)+"#"+g_camera_list[i].name+"#"+std::to_string(g_camera_list[i].index));
	}
	comboBoxCamera->signal_changed().connect(sigc::ptr_fun(onCameraChanged));

	ref_glade->get_widget("radioTypeLapangan", radioTypeLapangan);
	ref_glade->get_widget("radioTypeBola", radioTypeBola);
	ref_glade->get_widget("radioTypeGawangDepan", radioTypeGawangDepan);
	ref_glade->get_widget("radioTypeTemanDepan", radioTypeTemanDepan);
	ref_glade->get_widget("radioTypeMusuhDepan", radioTypeMusuhDepan);
	ref_glade->get_widget("radioTypeMusuh", radioTypeMusuh);
	ref_glade->get_widget("radioTypeTeman", radioTypeTeman);

	radioTypeLapangan->signal_toggled().connect(sigc::bind<int>(sigc::ptr_fun(setSegmentationMode), 0));
	radioTypeBola->signal_toggled().connect(sigc::bind<int>(sigc::ptr_fun(setSegmentationMode), 1));
	radioTypeGawangDepan->signal_toggled().connect(sigc::bind<int>(sigc::ptr_fun(setSegmentationMode), 2));
	radioTypeTemanDepan->signal_toggled().connect(sigc::bind<int>(sigc::ptr_fun(setSegmentationMode), 3));
	radioTypeMusuhDepan->signal_toggled().connect(sigc::bind<int>(sigc::ptr_fun(setSegmentationMode), 4));
	radioTypeMusuh->signal_toggled().connect(sigc::bind<int>(sigc::ptr_fun(setSegmentationMode), 5));
	radioTypeTeman->signal_toggled().connect(sigc::bind<int>(sigc::ptr_fun(setSegmentationMode), 6));

	ref_glade->get_widget("scaleHMin", scaleHMin);
	ref_glade->get_widget("scaleHMax", scaleHMax);
	ref_glade->get_widget("scaleSMin", scaleSMin);
	ref_glade->get_widget("scaleSMax", scaleSMax);
	ref_glade->get_widget("scaleVMin", scaleVMin);
	ref_glade->get_widget("scaleVMax", scaleVMax);

	scaleHMin->set_range(0.0,255.0);
	scaleHMin->signal_value_changed().connect([&]{
		hMin[g_segmentation_mode] = (int)scaleHMin->get_value();
	});
	scaleHMin->set_value(0.0);
	
	scaleHMax->set_range(0.0,255.0);
	scaleHMax->signal_value_changed().connect([&]{
		hMax[g_segmentation_mode] = (int)scaleHMax->get_value();
	});
	scaleHMax->set_value(255.0);

	scaleSMin->set_range(0.0,255.0);
	scaleSMin->signal_value_changed().connect([&]{
		sMin[g_segmentation_mode] = (int)scaleSMin->get_value();
	});
	scaleSMin->set_value(0.0);

	scaleSMax->set_range(0.0,255.0);
	scaleSMax->signal_value_changed().connect([&]{
		sMax[g_segmentation_mode] = (int)scaleSMax->get_value();
	});
	scaleSMax->set_value(255.0);

	scaleVMin->set_range(0.0,255.0);
	scaleVMin->signal_value_changed().connect([&]{
		vMin[g_segmentation_mode] = (int)scaleVMin->get_value();
	});
	scaleVMin->set_value(0.0);

	scaleVMax->set_range(0.0,255.0);
	scaleVMax->signal_value_changed().connect([&]{
		vMax[g_segmentation_mode] = (int)scaleVMax->get_value();
	});
	scaleVMax->set_value(255.0);

	ref_glade->get_widget("buttonImagePrev", imagePrev);
	ref_glade->get_widget("buttonImageNext", imageNext);
	ref_glade->get_widget("buttonImageSave", imageSave);

	imagePrev->signal_clicked().connect(sigc::ptr_fun(prevImage));
	imageNext->signal_clicked().connect(sigc::ptr_fun(nextImage));
	imageSave->signal_clicked().connect(sigc::ptr_fun(saveImage));

	ref_glade->get_widget("labelInfo", labelInfo);

	ref_glade->get_widget("buttonResetParameter", buttonResetParameter);
	ref_glade->get_widget("buttonReloadParameter", buttonReloadParameter);
	ref_glade->get_widget("buttonSaveParameter", buttonSaveParameter);

	buttonResetParameter->signal_clicked().connect(sigc::ptr_fun(resetParameter));
	buttonReloadParameter->signal_clicked().connect(sigc::ptr_fun(reloadParameter));
	buttonSaveParameter->signal_clicked().connect(sigc::ptr_fun(saveParameter));

    this->signal_hide().connect(sigc::ptr_fun(turnOffApp));
}

SegmentationUi::~SegmentationUi(){

}

void SegmentationUi::updateImage(cv::Mat &frameRGB, cv::Mat &frameHSV){
    if(!frameRGB.empty()) {
		this->imageRGB->set(Gdk::Pixbuf::create_from_data(frameRGB.data, Gdk::COLORSPACE_RGB, false, 8, frameRGB.cols, frameRGB.rows, frameRGB.step));
		this->imageRGB->queue_draw();
	}

    if(!frameHSV.empty()) {
		this->imageHSV->set(Gdk::Pixbuf::create_from_data(frameHSV.data, Gdk::COLORSPACE_RGB, false, 8, frameHSV.cols, frameHSV.rows, frameHSV.step));
		this->imageHSV->queue_draw();
	}
}