#!/bin/bash

if [ -z "$1" ]; then
		echo "cade o arquivo .bag?"
		exit -1
fi

echo "Carregando arquivo $(pwd)/$1 ... "

#precisei fazer assim pq o roslaunch nao sabe ler arquivo do PWD ...
roslaunch gpar_lidar cloud_offline.launch  bag_filename:="$(pwd)/$1"

