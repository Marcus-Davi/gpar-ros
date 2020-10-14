#!/bin/bash

catkin_make_isolated -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_CXX_FLAGS="-std=c++14"

