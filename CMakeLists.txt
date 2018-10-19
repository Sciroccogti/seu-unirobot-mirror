cmake_minimum_required(VERSION 3.0)
project(seu-unirobot)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} $ENV{CXXFLAGS} -O3 -std=c++11")

if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
    option(BUILD_TOOLS "Build Tools" ON)
else()
    option(BUILD_TOOLS "Build Tools" OFF)
endif()
add_subdirectory(src)
