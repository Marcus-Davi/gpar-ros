#!/bin/bash

catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --force-cmake -G"Eclipse CDT4 - Unix Makefiles"

