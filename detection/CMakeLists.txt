
cmake_minimum_required(VERSION 3.20)

project(RoboMasterDetection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV CONFIG REQUIRED)

set(ONNXRUNTIME_ROOT
    "$ENV{USERPROFILE}/RoboMaster/onnxruntime-win-x64-1.31.1"
)

# Main detection executable
add_executable(detection main.cpp)

target_include_directories(detection PRIVATE
    "${ONNXRUNTIME_ROOT}/include"
)

target_link_libraries(detection PRIVATE
    ${OpenCV_LIBS}
    "${ONNXRUNTIME_ROOT}/lib/onnxruntime.lib"
)

add_custom_command(TARGET detection POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${ONNXRUNTIME_ROOT}/lib/onnxruntime.dll"
    "$<TARGET_FILE_DIR:detection>"
)

# Visualization executable
add_executable(visualize visualize.cpp)

target_link_libraries(visualize PRIVATE
    ${OpenCV_LIBS}
)
