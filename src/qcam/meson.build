qcam_sources = files([
    'format_converter.cpp',
    'main.cpp',
    'main_window.cpp',
    '../cam/options.cpp',
    'qt_event_dispatcher.cpp',
    'viewfinder.cpp',
])

qcam_moc_headers = files([
    'main_window.h',
])

qt5 = import('qt5')
qt5_dep = dependency('qt5',
                     method : 'pkg-config',
                     modules : ['Core', 'Gui', 'Widgets'],
                     required : false)

if qt5_dep.found()
    moc_files = qt5.preprocess(moc_headers: qcam_moc_headers,
                               dependencies: qt5_dep)

    qcam  = executable('qcam', qcam_sources, moc_files,
                       install : true,
                       dependencies : [libcamera_dep, qt5_dep],
                       cpp_args : '-DQT_NO_KEYWORDS')
endif
