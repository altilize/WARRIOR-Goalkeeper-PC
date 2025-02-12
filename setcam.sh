#!/bin/sh
index_kamera=${1:-0}

echo "setting kamera $index_kamera ...."

#### WAKTU LOMBA NASIONAL
#v4l2-ctl -d /dev/video$index_kamera -c exposure_auto=1
#v4l2-ctl -d /dev/video$index_kamera -c exposure_absolute=400 
#v4l2-ctl -d /dev/video$index_kamera -c contrast=33
#v4l2-ctl -d /dev/video$index_kamera -c saturation=32 
#v4l2-ctl -d /dev/video$index_kamera -c gain=1
#v4l2-ctl -d /dev/video$index_kamera -c brightness=129
#v4l2-ctl -d /dev/video$index_kamera -c backlight_compensation=0 
#v4l2-ctl -d /dev/video$index_kamera -c white_balance_temperature_auto=0
#v4l2-ctl -d /dev/video$index_kamera -c white_balance_temperature=5640
#v4l2-ctl -d /dev/video$index_kamera -c focus_auto=0


# v4l2-ctl -d /dev/video$index_kamera -c exposure_auto=3
# v4l2-ctl -d /dev/video$index_kamera -c exposure_absolute=0 
# v4l2-ctl -d /dev/video$index_kamera -c contrast=121
# v4l2-ctl -d /dev/video$index_kamera -c saturation=128 
# v4l2-ctl -d /dev/video$index_kamera -c gain=0
# v4l2-ctl -d /dev/video$index_kamera -c brightness=15
# v4l2-ctl -d /dev/video$index_kamera -c backlight_compensation=0 
# v4l2-ctl -d /dev/video$index_kamera -c white_balance_temperature_auto=5
# v4l2-ctl -d /dev/video$index_kamera -c white_balance_temperature=4681
# v4l2-ctl -d /dev/video$index_kamera -c focus_auto=0



### LAB ROBOT
v4l2-ctl -d /dev/video$index_kamera -c exposure_auto=1
v4l2-ctl -d /dev/video$index_kamera -c exposure_absolute=400 
v4l2-ctl -d /dev/video$index_kamera -c contrast=45
v4l2-ctl -d /dev/video$index_kamera -c saturation=119 
v4l2-ctl -d /dev/video$index_kamera -c gain=43 
v4l2-ctl -d /dev/video$index_kamera -c brightness=120
v4l2-ctl -d /dev/video$index_kamera -c backlight_compensation=0 
v4l2-ctl -d /dev/video$index_kamera -c white_balance_temperature_auto=0 
v4l2-ctl -d /dev/video$index_kamera -c white_balance_temperature=5700
v4l2-ctl -d /dev/video$index_kamera -c focus_auto=0