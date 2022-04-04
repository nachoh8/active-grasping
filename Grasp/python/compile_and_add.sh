#!/bin/bash

# Compile

if [! -d build ]
then
    mkdir build
fi

cd build

cmake ..
if [ $? -ne 0 ]
then
    echo "Error on cmake"
    exit 1
fi

make
if [ $? -ne 0 ]
then
    echo "Error on make"
    exit 2
fi

# Copy module to python site packages

PYGRASP_FILE="pygrasp.py"
PYGRASP_LIB="_pygrasp.so"

PYGRASP_MODULE_DIR="${HOME}/.local/lib/python3.6/site-packages/pygrasp"

mkdir $PYGRASP_MODULE_DIR

cp $PYGRASP_FILE $PYGRASP_LIB -t $PYGRASP_MODULE_DIR
