#!/bin/bash

#-d : debug
#-r : release


if [ $1 == "-d" ]; then
    cmake -DCMAKE_BUILD_TYPE=Debug
else
    cmake -DCMAKE_BUILD_TYPE=Release
fi    

make -j8
