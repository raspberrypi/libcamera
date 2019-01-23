cam_sources = files([
    'event_loop.cpp',
    'main.cpp',
    'options.cpp',
])

cam  = executable('cam', cam_sources,
                  link_with : libcamera,
                  include_directories : libcamera_includes)
