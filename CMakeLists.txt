cmake_minimum_required(VERSION 3.21)
project(warrior)

find_package(OpenCV REQUIRED)
FIND_PACKAGE(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
link_directories(${GTKMM_LIBRARY_DIRS})
include_directories(${GTKMM_INCLUDE_DIRS})

add_subdirectory(gui)

# Set standard c++ compile option
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(kamera kamera.cpp library/Camera.cpp gui/PilihKamera.cpp gui/KalibrasiKamera.cpp)
target_link_libraries(kamera ${OpenCV_LIBS} ${GTKMM_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(segmentasi segmentasi.cpp library/Camera.cpp library/Citra.cpp)
target_link_libraries(segmentasi ${OpenCV_LIBS} ${GTKMM_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} segmentation)

# add_executable( main main.cpp lib/Algoritma.cpp lib/kalmanFilter.cpp lib/KomunikasiUDP.cpp lib/KomunikasiArduino.cpp lib/InputChar.cpp lib/Utility.cpp lib/Citra.cpp lib/Camera.cpp lib/ObstaclesDetector.cpp lib/KomunikasiRobot.cpp)
add_executable(main main.cpp library/Camera.cpp library/Citra.cpp library/InisiasiSerial.cpp library/KomunikasiMikro.cpp library/UDPListen.cpp library/UDPSend.cpp library/Algoritma.cpp library/Motion.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})