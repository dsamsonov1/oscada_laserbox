cmake_minimum_required(VERSION 3.15)
project(oscada_laserbox)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(oscada_laserbox
        CMakeLists.txt
        epiLaser.cpp
        epiLaser.h
        ftd2xx.h
        instl.sh
        module.cpp
        module.h
        WinTypes.h)
