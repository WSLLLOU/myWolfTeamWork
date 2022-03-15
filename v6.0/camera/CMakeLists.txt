# Create project
cmake_minimum_required(VERSION 3.10.0)
project(WolfVision)
set(CMAKE_CXX_STANDARD 17)

# Find and add opencv as required
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

# fmt
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/fmt)
# Add fmtlib directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/fmt/include)

# # add_lib libMVSDK.so

# Camera
add_library(mv_video_capture mv_video_capture.cpp)
include_directories(camera)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/utils)

target_link_libraries(mv_video_capture
    PUBLIC
    fmt::fmt
    ${OpenCV_LIBS}
    ${CMAKE_CURRENT_SOURCE_DIR}/x64/libMVSDK.so
)