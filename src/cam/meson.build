cam_sources = files([
    'buffer_writer.cpp',
    'capture.cpp',
    'event_loop.cpp',
    'main.cpp',
    'options.cpp',
])

cam  = executable('cam', cam_sources,
                  dependencies : libcamera_dep,
                  install : true)
