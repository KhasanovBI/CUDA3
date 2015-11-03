#!/bin/bash
ROS_PACKAGE_PATH=/home/bulat/temp/CUDA2_OpenCV_ROS/Project/ros_workspace/tutorial:$ROS_PACKAGE_PATH
cmake CMakeLists.txt
make
