cam_sources = files([
    'buffer_writer.cpp',
    'capture.cpp',
    'event_loop.cpp',
    'main.cpp',
    'options.cpp',
    'stream_options.cpp',
])

cam  = executable('cam', cam_sources,
                  dependencies : [ libatomic, libcamera_dep ],
                  install : true)
