# Bench tool: rangings of the front sensor on the USB console (see main/main.cpp).
# Only the controller component of the robot firmware: no link, no micro-ROS.
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ../../components/controller)
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(front_range)
