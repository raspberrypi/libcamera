# SPDX-License-Identifier: CC0-1.0

qt5 = import('qt5')
qt5_dep = dependency('qt5',
                     method : 'pkg-config',
                     modules : ['Core', 'Gui', 'Widgets'],
                     required : get_option('qcam'),
                     version : '>=5.4')

if not qt5_dep.found()
    qcam_enabled = false
    subdir_done()
endif

qcam_enabled = true

qcam_sources = files([
    'cam_select_dialog.cpp',
    'format_converter.cpp',
    'main.cpp',
    'main_window.cpp',
    'message_handler.cpp',
    'viewfinder_qt.cpp',
])

qcam_moc_headers = files([
    'cam_select_dialog.h',
    'main_window.h',
    'viewfinder_qt.h',
])

qcam_resources = files([
    'assets/feathericons/feathericons.qrc',
])

qt5_cpp_args = [apps_cpp_args, '-DQT_NO_KEYWORDS']

if cxx.has_header_symbol('QOpenGLWidget', 'QOpenGLWidget',
                         dependencies : qt5_dep, args : '-fPIC')
    qcam_sources += files([
        'viewfinder_gl.cpp',
    ])
    qcam_moc_headers += files([
        'viewfinder_gl.h',
    ])
    qcam_resources += files([
        'assets/shader/shaders.qrc'
    ])
endif

# gcc 9 introduced a deprecated-copy warning that is triggered by Qt until
# Qt 5.13. clang 10 introduced the same warning, but detects more issues
# that are not fixed in Qt yet. Disable the warning manually in both cases.
if ((cc.get_id() == 'gcc' and cc.version().version_compare('>=9.0') and
     qt5_dep.version().version_compare('<5.13')) or
    (cc.get_id() == 'clang' and cc.version().version_compare('>=10.0')))
    qt5_cpp_args += ['-Wno-deprecated-copy']
endif

resources = qt5.preprocess(moc_headers: qcam_moc_headers,
                           qresources : qcam_resources,
                           dependencies: qt5_dep)

qcam  = executable('qcam', qcam_sources, resources,
                   install : true,
                   link_with : apps_lib,
                   dependencies : [
                       libatomic,
                       libcamera_public,
                       libtiff,
                       qt5_dep,
                   ],
                   cpp_args : qt5_cpp_args)
