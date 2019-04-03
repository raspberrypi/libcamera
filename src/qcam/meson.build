qcam_sources = files([
    'format_converter.cpp',
    'main.cpp',
    'main_window.cpp',
    '../cam/options.cpp',
    'qt_event_dispatcher.cpp',
    'viewfinder.cpp',
])

import('qt5')
qt5_dep = dependency('qt5',
                     method : 'pkg-config',
                     modules : ['Core', 'Gui', 'Widgets'],
                     required : false)

if qt5_dep.found()
    qcam  = executable('qcam', qcam_sources,
                       link_with : libcamera,
                       include_directories : libcamera_includes,
                       dependencies : qt5_dep,
                       cpp_args : '-DQT_NO_KEYWORDS')
endif
