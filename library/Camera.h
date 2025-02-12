#ifndef CAMERA_H
#define CAMERA_H

#include "TypeData.h"
#include <string>
#include <vector>

using namespace std;

namespace R2CCamera{
    void loadCameras(vector<Camera>&);
    Camera loadCameraByPath(string);
    int getCameraIndexByPath(string);
    void getCameraParameter(Camera&);
    int saveCameraParameterToFile(Camera);
    Camera loadCameraParameterFromFile(Camera&);
    void printCameraInfo(Camera);
    void setParameterCamera(Camera);
    void setParameterCameraByparameter(int, Parameter);
}

#endif