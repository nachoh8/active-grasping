#!/bin/bash

PYTHON_2=$1

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

PYGRASP_MODULE_DIR="/usr/lib/python2.7/dist-packages"

if [ $PYTHON_2 -eq 0 ] # is Python3
then
    PYGRASP_MODULE_DIR="${HOME}/.local/lib/python3.6/site-packages"
    # mkdir $PYGRASP_MODULE_DIR
fi

echo "Copying pygrasp module to ${PYGRASP_MODULE_DIR}"

sudo cp $PYGRASP_FILE $PYGRASP_LIB -t $PYGRASP_MODULE_DIR
