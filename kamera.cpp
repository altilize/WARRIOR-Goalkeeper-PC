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

#include <gtkmm.h>
#include "gui/PilihKamera.h"
#include "gui/KalibrasiKamera.h"

using namespace std;
using namespace cv;

vector<Camera> cameraList;
Camera selectedCamera;
int success = 0;

Gtk::Label* parameterLabel;

Mat frame;
VideoCapture cap;
std::mutex imageMutex;
Glib::Dispatcher dispatcher;

void selectCamera(Camera cam, Gtk::Window* window){
    selectedCamera = cam;
    success = 1;

    window->close();
}

void captureImage(){
    while(success){
        cap >> frame;
        imageMutex.lock();
        cv::cvtColor(frame, frame, COLOR_BGR2RGB);
        imageMutex.unlock();
        dispatcher.emit();

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

int main(int argc, char *argv[]){
    R2CCamera::loadCameras(cameraList);
    printf("Camera size: %li\n\n", cameraList.size());

    auto app = Gtk::Application::create( argc, argv, "r2c.warrior.camera" );

    SelectCamera selectCameraWindow;

    int result = app->run(selectCameraWindow);

    if(success == 1){

        R2CCamera::setParameterCamera(selectedCamera);
        cap.open(selectedCamera.index, cv::CAP_V4L2);
        cap.set(CAP_PROP_FRAME_WIDTH, 640);
		cap.set(CAP_PROP_FRAME_HEIGHT, 480);

        app = Gtk::Application::create( argc, argv, "r2c.warrior.camera" );
        Glib::RefPtr<Gtk::Builder> builder = Gtk::Builder::create_from_file("ui/camera_ui.glade"); 
        
        CameraCallibration* cameraCallibration;
        builder->get_widget_derived("CallibrationWindow", cameraCallibration); 
        cameraCallibration->setCamera(selectedCamera);

        dispatcher.connect([&]() {
            imageMutex.lock();
            cameraCallibration->updateImage(frame);
            imageMutex.unlock();
        });

        std::thread cameraThread = std::thread(&captureImage);
        app->run(*cameraCallibration);
		cameraThread.join();
    }
}


void updateInfo(){
    string parameterInfo;
    for(auto params: selectedCamera.parameters){
        Parameter param = params.second;
        parameterInfo+=param.name;
        parameterInfo+=": ";
        parameterInfo+="value: ";
        parameterInfo+=std::to_string(param.value);
        parameterInfo+="\n\tdefaultValue: ";
        parameterInfo+=std::to_string(param.defaultValue);
        parameterInfo+="\tmin: ";
        parameterInfo+=std::to_string(param.min);
        parameterInfo+="\tmax: ";
        parameterInfo+=std::to_string(param.max);
        parameterInfo+="\n";
    }
    parameterLabel->set_label(parameterInfo);
}

void setParameter(string name, int value){
    Parameter param = selectedCamera.parameters[name];
    
    param.value = param.value+value;

    if(param.value > param.max){
        param.value = param.max;
    }
    if(param.value < param.min){
        param.value = param.min;
    }
    selectedCamera.parameters[param.name] = param;
    R2CCamera::setParameterCameraByparameter(selectedCamera.index, param);
    updateInfo();
}

void resetParameter(){
    std::cout << "Reset parameter" << std::endl;
    for(auto params: selectedCamera.parameters){
        Parameter param = params.second;
        param.value = param.defaultValue;
        selectedCamera.parameters[param.name] = param;
    }
    R2CCamera::setParameterCamera(selectedCamera);

    updateInfo();
}

void reloadParameter(){
    std::cout << "Reload parameter from file" << std::endl;

    R2CCamera::loadCameraParameterFromFile(selectedCamera);
    R2CCamera::setParameterCamera(selectedCamera);

    updateInfo();
}

void saveParameter(){
    std::cout << "Save parameter to file" << std::endl;

    R2CCamera::saveCameraParameterToFile(selectedCamera);
}

void turnOffApp(){
    success = 0;
}