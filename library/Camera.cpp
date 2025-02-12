#include "Camera.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>

void R2CCamera::loadCameras(vector<Camera>& cameraList){
    printf("Getting all cameras...\n");
    char path[1024];
    FILE *fp;
    string ls = "ls /dev/v4l/by-id/";
    fp = popen(ls.c_str(), "r");
    if(fp == NULL)
	{
		printf("failed to run command load cameras\n");
		return;
	}
    while(fscanf(fp, "%s", path) != EOF){
        printf("Detected camera: %s\n", path);
        Camera camera = Camera();
        camera.name = string(path);

        string ls = "/dev/v4l/by-id/";
        ls.append(path);
        camera.path = ls;

        camera.index = R2CCamera::getCameraIndexByPath(camera.path);

        R2CCamera::getCameraParameter(camera);

        if(camera.parameters.size() > 0){
            R2CCamera::loadCameraParameterFromFile(camera);
            cameraList.push_back(camera);
        }
    }
    printf("\n");
	pclose(fp);
}

Camera R2CCamera::loadCameraByPath(string path){
    
    std::vector<std::string> strings;
    size_t start;
    size_t end = 0;
    while ((start = path.find_first_not_of("/", end)) != std::string::npos) {
        end = path.find("/", start);
        strings.push_back(path.substr(start, end - start));
    }

    Camera camera = Camera();
    camera.name = strings[strings.size() - 1];
    camera.path = path;
    camera.index = getCameraIndexByPath(camera.path);
    R2CCamera::getCameraParameter(camera);

    if(camera.parameters.size() > 0){
        R2CCamera::loadCameraParameterFromFile(camera);
    }

    return camera;
}

int R2CCamera::getCameraIndexByPath(string cameraPath){
    FILE *fp;
	char path[1024];
    path[0] = '\0';

	string command = "/bin/ls -l ";
	command += cameraPath;
	command += " 2>/dev/null | /usr/bin/awk {'print $ll'}";

	fp = popen(command.c_str(),"r");
	if(fp == NULL)
	{
		printf("failed to run command get camera index\n");
		return 0;
	}

	while(fgets(path,sizeof(path)-1,fp) != NULL)
	{

	}
	pclose(fp);
	
    int index = 0;
    if(strlen(path) > 1){
        index = path[strlen(path)-2]-'0';
        if(index<0){
            return 0;
        }
        return index;
    }
    return index;
}

void R2CCamera::getCameraParameter(Camera& cam){
	string command = "v4l2-ctl -l -d /dev/video";
	command += std::to_string(cam.index);

	FILE* fp = popen(command.c_str(),"r");
	if(fp == NULL)
	{
		printf("failed to run command get camera parameter\n");
		return;
	}

    char path[1024];
	while(fscanf(fp, " %[^\n]\n", path) != EOF)
	{
        Parameter parameter;
        char paramName[25];
        char temp[100];
        int min = 0;
        int max = 1;
        int step = 1;
        int defaultValue = 0;
        int value = 0;
        sscanf(path, "%[^ 0] 0%[^\n]\n",paramName, temp);
        parameter.name = string(paramName);

        sscanf(path, "%[^d\n]default=%i%[^v\n]value=%i%[^\n]\n",temp, &defaultValue, temp, &value, temp);

        parameter.defaultValue = defaultValue;
        parameter.value = value;

        if(strstr(path, "min") != NULL){
            sscanf(path, "%[^:\n]:%[^m\n]min=%i%[^\n]\n", temp, temp, &min, temp);
        }
        if(strstr(path, "max") != NULL){
            sscanf(path, "%[^:\n]:%[^m\n]min%[^m\n]max=%i%[^\n]\n", temp, temp, temp, &max, temp);
        }
        if(strstr(path, "step") != NULL){
            sscanf(path, "%[^:\n]:%[^s\n]step=%i%[^\n]\n", temp, temp, &step, temp);
        }

        parameter.min = min;
        parameter.max = max;
        parameter.step = step;

        cam.parameters[parameter.name] = parameter;
	}
	pclose(fp);
}

int R2CCamera::saveCameraParameterToFile(Camera cam){
    system("mkdir datacamera 2>/dev/null");
    string filePath = "datacamera/"+cam.name+".txt";
    FILE* save = fopen(filePath.c_str(), "w");
    for(auto params: cam.parameters){
        string paramName = params.first;
        Parameter param = params.second;
        fprintf(save, "%s=%i\n", param.name.c_str(), param.value);
    }
    fclose(save);
    return 0;
}

Camera R2CCamera::loadCameraParameterFromFile(Camera& cam){
    string filePath = "datacamera/"+cam.name+".txt";
    FILE* saved;
    if(saved = fopen(filePath.c_str(), "r")){
        printf("Read saved file from camera %s\n", cam.name.c_str());
        char paramName[100];
        int value;
        while(fscanf(saved, "%[^=]=%i\n", paramName, &value) != EOF){
            if(cam.parameters.find(paramName) != cam.parameters.end()){
                Parameter newParam = cam.parameters.at(paramName);
                newParam.value = value;
                cam.parameters[paramName] = newParam;
            }
        }
        fclose(saved);
    }
    return cam;
}

void R2CCamera::printCameraInfo(Camera selectedCamera){
    printf("%s\n",selectedCamera.name.c_str());
    printf("Index: %i, path: %s\n", selectedCamera.index, selectedCamera.path.c_str());
    printf("Parameter size: %li\n", selectedCamera.parameters.size());
    for(auto params: selectedCamera.parameters){
        string paramName = params.first;
        Parameter param = params.second;
        printf("\t%s: defaultValue=%i value=%i min=%i max=%i step=%i\n", paramName.c_str(), param.defaultValue, param.value, param.min, param.max, param.step);
    }
    printf("\n");
}

void R2CCamera::setParameterCamera(Camera cam){
    for(auto params: cam.parameters){
        Parameter param = params.second;
        string command = "v4l2-ctl -d /dev/video"+std::to_string(cam.index)+" -c "+param.name+"="+std::to_string(param.value);
        system(command.c_str());
        usleep(10*1000);
    }   
}

void R2CCamera::setParameterCameraByparameter(int videoId, Parameter param){
    string command = "v4l2-ctl -d /dev/video"+std::to_string(videoId)+" -c "+param.name+"="+std::to_string(param.value);
    system(command.c_str());
}