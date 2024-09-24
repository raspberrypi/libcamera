# SPDX-License-Identifier: CC0-1.0

qt6 = import('qt6')
qt6_dep = dependency('qt6',
                     method : 'pkg-config',
                     modules : ['Core', 'Gui', 'OpenGL', 'OpenGLWidgets', 'Widgets'],
                     required : get_option('qcam'),
                     version : '>=6.2')

if not qt6_dep.found()
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
    'viewfinder_gl.cpp',
    'viewfinder_qt.cpp',
])

qcam_moc_headers = files([
    'cam_select_dialog.h',
    'main_window.h',
    'viewfinder_gl.h',
    'viewfinder_qt.h',
])

qcam_resources = files([
    'assets/feathericons/feathericons.qrc',
    'assets/shader/shaders.qrc',
])

qt6_cpp_args = [
    apps_cpp_args,
    '-DQT_NO_KEYWORDS',
    '-Wno-extra-semi',
]

resources = qt6.preprocess(moc_headers : qcam_moc_headers,
                           qresources : qcam_resources,
                           dependencies : qt6_dep)

qcam  = executable('qcam', qcam_sources, resources,
                   install : true,
                   install_tag : 'bin',
                   link_with : apps_lib,
                   dependencies : [
                       libatomic,
                       libcamera_public,
                       libtiff,
                       qt6_dep,
                   ],
                   cpp_args : qt6_cpp_args)
