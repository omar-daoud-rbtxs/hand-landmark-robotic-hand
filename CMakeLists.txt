cmake_minimum_required(VERSION 3.10)

project(HandLandmark)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Search for the core TFLite library
find_library(TFLITE_LIB
    NAMES tensorflow-lite libtensorflow-lite.a
    PATHS
        /usr/local/lib
        /usr/lib
        /home/omar/tflite_build
)

if(NOT TFLITE_LIB)
    message(FATAL_ERROR "TFLite library not found. Run 'sudo cmake --install .' inside ~/tflite_build")
endif()

# Grab all the dependency static libraries
file(GLOB_RECURSE TFLITE_DEP_LIBS 
    "/home/omar/tflite_build/_deps/*.a"
    "/home/omar/tflite_build/pthreadpool/*.a"
)

# --- THE FIX ---
# 1. Strip out ALL fft2d libraries from the automatic list to avoid conflicts
list(FILTER TFLITE_DEP_LIBS EXCLUDE REGEX "fft2d-build")

# 2. Manually add back ONLY the two specific fft2d libraries TFLite needs
list(APPEND TFLITE_DEP_LIBS
    "/home/omar/tflite_build/_deps/fft2d-build/libfft2d_fftsg.a"
    "/home/omar/tflite_build/_deps/fft2d-build/libfft2d_fftsg2d.a"
)
# ---------------

add_executable(h_landmark h_landmark.cpp)

target_include_directories(h_landmark PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    /home/omar/tensorflow
    /usr/local/include
)

target_link_libraries(h_landmark 
    PRIVATE
    ${OpenCV_LIBS}
    gpiodcxx
    -Wl,--start-group
    ${TFLITE_LIB}
    ${TFLITE_DEP_LIBS}
    -Wl,--end-group
    Threads::Threads
    dl
)