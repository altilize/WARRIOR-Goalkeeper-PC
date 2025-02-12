#ifndef STRIKERCAMERA_H
#define STRIKERCAMERA_H

#include <iostream>

/*Back*/
const std::string KAMERA_depan = "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_7FB19ADF-video-index0";
const std::string KAMERA_omni = "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_D0DC2E9E-video-index0";

struct parameter_frame{
    int geser_frame_x = 4;
    int geser_frame_y = 10;
    int radius_mask = 165;
    float sudut_putaran_omni = -129;
}ParameterFrame;

#endif