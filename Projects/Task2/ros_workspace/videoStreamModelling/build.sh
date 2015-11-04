#!/bin/bash
ROS_PACKAGE_PATH=$(pwd):$ROS_PACKAGE_PATH
cmake CMakeLists.txt
make
