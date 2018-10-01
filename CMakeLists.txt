cmake_minimum_required(VERSION 3.0)
project(seu-unirobot)

# add_definitions("-std=c++11")
set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
option(BUILD_TOOLS "Build Tools" ON)

add_subdirectory(src)
