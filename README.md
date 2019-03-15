# seu-unirobot

## Description

This project contains a controller, some debug tools and some scripts for seu-robocup kidszie team. The controller and debug tools are written in c++. The scripts are written in python.

## Build tools
* cmake >= 3.12
* gcc
* g++
* nvcc

## Dependencies

### c++ libraries
* cuda >= 9.0
* cudnn >= 7.0
* opencv >= 3.3.1
* libeigen3-dev
* libboost-all-dev
* freeglut3-dev
* libv4l-dev
* MVSDK (library for our camera)
* qt5-default
* astyle

You can use Jetpack-3.3 to install cuda, opencv and cross compiler tools.

### python3 libraries
* ssh2-python

## Compile & Run

### Compile for x86_64
* cd path/to/project
* ./x86_64-build.py
* Then you find the executable files in bin/x86_64
* You can run with ./exe_name -h to get infomation about how to use

### Cross compile for aarch64
* cd path/to/project
* ./aarch64-build.py
* Then you find the executable files in bin/aarch64
* If you want to run program, you should connect with robot, then use the script start_robot.py in bin/

## Recommend OS
* ubuntu 16.04 64bit

## Recommend IDE
* Visual Studio Code(with c++ and python plugin)
