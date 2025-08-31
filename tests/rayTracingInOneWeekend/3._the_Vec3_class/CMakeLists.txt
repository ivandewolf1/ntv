cmake_minimum_required(VERSION 3.31)
project(freshstart)

set(CMAKE_CXX_STANDARD 20)
include_directories(./../../../ntv/src/) ## extra depth in dir strucrture
add_executable(freshstart main.cpp)
