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
    qt5_cpp_args = [ '-DQT_NO_KEYWORDS' ]

    # gcc 9 introduced a deprecated-copy warning that is triggered by Qt until
    # Qt 5.13. Disable it manually.
    if cc.get_id() == 'gcc'
        gcc_version = cc.version().split('.')
        qt5_version = qt5_dep.version().split('.')
        if qt5_version[1].to_int() < 13 and gcc_version[0].to_int() >= 9
            qt5_cpp_args += [ '-Wno-deprecated-copy' ]
        endif
    endif

    moc_files = qt5.preprocess(moc_headers: qcam_moc_headers,
                               dependencies: qt5_dep)

    qcam  = executable('qcam', qcam_sources, moc_files,
                       install : true,
                       dependencies : [libcamera_dep, qt5_dep],
                       cpp_args : qt5_cpp_args)
endif
