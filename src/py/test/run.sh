#!/bin/bash

# set frame rate
#v4l2-ctl -d0 -p60

export LIBCAMERA_LOG_LEVELS=*:3
export PYTHONPATH=../../../build/src/py:/home/tomba/work/kmsxx/build/debug/py
export QT_XCB_GL_INTEGRATION="xcb_egl"
#export QT_LOGGING_RULES="qt.qpa.gl=true"

python3 $*
