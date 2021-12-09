#!/bin/bash

export LIBCAMERA_LOG_LEVELS=*:3
export PYTHONPATH=../../../build/debug/src/py

python3 -m pdb $*
