libcamera_gst_sources = [
    'gstlibcamera-utils.cpp',
    'gstlibcamera.c',
    'gstlibcamerapad.cpp',
    'gstlibcameraprovider.cpp',
    'gstlibcamerasrc.cpp',
]

libcamera_gst_c_args = [
    '-DVERSION="@0@"'.format(libcamera_git_version),
    '-DPACKAGE="@0@"'.format(meson.project_name()),
]

glib_dep = dependency('glib', required : get_option('gstreamer'))

gst_dep = dependency('gstreamer-video-1.0', version : '>=1.14.0',
                     required : get_option('gstreamer'))

if glib_dep.found() and gst_dep.found()
    # The G_DECLARE_FINAL_TYPE macro creates static inline functions that were
    # not marked as possibly unused prior to GLib v2.63.0. This causes clang to
    # complain about the ones we are not using. Silence the -Wunused-function
    # warning in that case.
    if cc.get_id() == 'clang' and glib_dep.version().version_compare('<2.63.0')
        libcamera_gst_c_args += [ '-Wno-unused-function' ]
    endif

    libcamera_gst = shared_library('gstlibcamera',
        libcamera_gst_sources,
        c_args : libcamera_gst_c_args,
        dependencies : [libcamera_dep, gst_dep],
        install: true,
        install_dir : '@0@/gstreamer-1.0'.format(get_option('libdir')),
    )
endif
