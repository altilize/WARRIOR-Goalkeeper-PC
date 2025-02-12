cmake_minimum_required(VERSION 3.21)
project(gui)

find_package(OpenCV REQUIRED)
FIND_PACKAGE(PkgConfig REQUIRED)

pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
link_directories(${GTKMM_LIBRARY_DIRS})
include_directories(${GTKMM_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(segmentation src/SegmentationUi.cpp)
target_include_directories(
        segmentation
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(segmentation ${OpenCV_LIBS}  ${GTKMM_LIBRARIES})