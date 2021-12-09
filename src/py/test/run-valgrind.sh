#!/bin/bash

export LIBCAMERA_LOG_LEVELS=*:3
export PYTHONMALLOC=malloc
export PYTHONPATH=../../../build/debug/src/py

valgrind --suppressions=/usr/lib/valgrind/python3.supp --suppressions=valgrind-pycamera.supp --leak-check=full --show-leak-kinds=definite --gen-suppressions=yes python3-dbg $*
