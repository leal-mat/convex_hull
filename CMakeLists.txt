cmake_minimum_required(VERSION 3.10)
project(gw3d)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(gw3d main.cpp)