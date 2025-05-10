#ifndef BACKCAMERA_H
#define BACKCAMERA_H

#include <iostream>


//  ---------- Robot Kiper --------------------
// const std::string KAMERA_depan = "/dev/v4l/by-id/usb-SunplusIT_Inc_HD_Webcam-video-index0";
// const std::string KAMERA_omni = "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_A9E0566F-video-index0";

// ---------- Robot Roda 4 ------------------
const std::string KAMERA_depan = "/dev/v4l/by-id/usb-SunplusIT_Inc_HD_Webcam-video-index0";
const std::string KAMERA_omni = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0179-video-index0";

struct parameter_frame{
    int geser_frame_x = -2;//15 roda 4 // 0 roda 3
    int geser_frame_y = 10;//-12 roda 4 // 0 roda 3
    int radius_mask = 165;
    float sudut_putaran_omni = 140;//0.2 roda 4 // 150 roda 3
}ParameterFrame;

// struct parameter_frame{
//     int geser_frame_x = 4;
//     int geser_frame_y = 10;
//     int radius_mask = 165;
//     float sudut_putaran_omni = -129;
// }ParameterFrame;


#endif