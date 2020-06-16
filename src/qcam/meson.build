# SPDX-License-Identifier: CC0-1.0

qcam_sources = files([
    '../cam/options.cpp',
    '../cam/stream_options.cpp',
    'format_converter.cpp',
    'main.cpp',
    'main_window.cpp',
    'viewfinder.cpp',
])

qcam_moc_headers = files([
    'main_window.h',
    'viewfinder.h',
])

qcam_resources = files([
    'assets/feathericons/feathericons.qrc',
])

qt5 = import('qt5')
qt5_dep = dependency('qt5',
                     method : 'pkg-config',
                     modules : ['Core', 'Gui', 'Widgets'],
                     required : get_option('qcam'))

if qt5_dep.found()
    qcam_deps = [
        libcamera_dep,
        qt5_dep,
    ]

    qt5_cpp_args = [ '-DQT_NO_KEYWORDS' ]

    tiff_dep = dependency('libtiff-4', required : false)
    if tiff_dep.found()
        qt5_cpp_args += [ '-DHAVE_TIFF' ]
        qcam_deps += [ tiff_dep ]
        qcam_sources += files([
            'dng_writer.cpp',
        ])
    endif

    # gcc 9 introduced a deprecated-copy warning that is triggered by Qt until
    # Qt 5.13. clang 10 introduced the same warning, but detects more issues
    # that are not fixed in Qt yet. Disable the warning manually in both cases.
    if ((cc.get_id() == 'gcc' and cc.version().version_compare('>=9.0') and
         qt5_dep.version().version_compare('<5.13')) or
        (cc.get_id() == 'clang' and cc.version().version_compare('>=10.0')))
        qt5_cpp_args += [ '-Wno-deprecated-copy' ]
    endif

    resources = qt5.preprocess(moc_headers: qcam_moc_headers,
                               qresources : qcam_resources,
                               dependencies: qt5_dep)

    qcam  = executable('qcam', qcam_sources, resources,
                       install : true,
                       dependencies : qcam_deps,
                       cpp_args : qt5_cpp_args)
endif
