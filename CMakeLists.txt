cmake_minimum_required(VERSION 3.29)
project(vpCalibrate)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(vpCalibrate main.cpp
        util.cpp
        util.h)

target_link_libraries(vpCalibrate ${OpenCV_LIBS})


