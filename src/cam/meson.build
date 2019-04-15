cam_sources = files([
    'buffer_writer.cpp',
    'event_loop.cpp',
    'main.cpp',
    'options.cpp',
])

cam  = executable('cam', cam_sources,
                  link_with : libcamera,
                  install : true,
                  include_directories : libcamera_includes)
