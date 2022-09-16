cmake_minimum_required(VERSION 2.6)

project(main)

add_definitions(-std=c++17)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# find cppzmq wrapper, installed by make of cppzmq
find_package(cppzmq)

add_executable(receive receive.cpp)
target_link_libraries(receive cppzmq)
