# mkdir build
# cd build
# cmake  -DCMAKE_BUILD_TYPE=Release ..
# make
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
cmake_minimum_required (VERSION 2.8) 
project (tennis)
set(raspicam_DIR "/usr/local/lib/cmake")
find_package(raspicam REQUIRED)
find_package(OpenCV)
IF  ( OpenCV_FOUND AND raspicam_CV_FOUND)
MESSAGE(STATUS "COMPILING...")
add_executable (tennis utils.cpp led.cpp config.cpp component.cpp vision.cpp location.cpp motor.cpp observer.cpp picker.cpp main.cpp)  
target_link_libraries (tennis wiringPi wiringPiDev pthread ${raspicam_CV_LIBS})
ELSE()
MESSAGE(FATAL_ERROR "OPENCV NOT FOUND IN YOUR SYSTEM")
ENDIF()
