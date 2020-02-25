cmake_minimum_required(VERSION 3.15)

project(oscada_laserbox_module)

set(CMAKE_CXX_STANDARD 11)

include_directories(
        /usr/include/openscada
        /usr/include/jsoncpp
        /usr/local/lib
        /usr/lib
        )

link_directories(/usr/lib/ /usr/local/lib)

#add_executable(oscada_laserbox_standalone lasertest.cpp epiLaser.cpp epiLaser.h  WinTypes.h ftd2xx.h)
#target_link_libraries(oscada_laserbox ftd2xx jsoncpp)

add_library(oscada_laserbox_module SHARED module.cpp module.h ftd2xx.h WinTypes.h epiLaser.h epiLaser.cpp)
target_link_libraries(oscada_laserbox_module ftd2xx jsoncpp)

