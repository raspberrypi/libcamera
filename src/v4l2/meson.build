# SPDX-License-Identifier: CC0-1.0

v4l2_enabled = get_option('v4l2').allowed()

if not v4l2_enabled
    subdir_done()
endif

v4l2_compat_sources = files([
    'v4l2_camera.cpp',
    'v4l2_camera_file.cpp',
    'v4l2_camera_proxy.cpp',
    'v4l2_compat.cpp',
    'v4l2_compat_manager.cpp',
])

v4l2_compat_cpp_args = [
    # Meson enables large file support unconditionally, which redirect file
    # operations to 64-bit versions. This results in some symbols being
    # renamed, for instance open() being renamed to open64(). As the V4L2
    # adaptation wrapper needs to provide both 32-bit and 64-bit versions of
    # file operations, disable transparent large file support.
    '-U_FILE_OFFSET_BITS',
    '-D_FILE_OFFSET_BITS=32',
    '-D_LARGEFILE64_SOURCE',
    '-U_TIME_BITS',
    '-fvisibility=hidden',
]

v4l2_compat = shared_library('v4l2-compat',
                             v4l2_compat_sources,
                             name_prefix : '',
                             install : true,
                             install_dir : libcamera_libexecdir,
                             dependencies : [libcamera_private, libdl],
                             cpp_args : v4l2_compat_cpp_args)

# Provide a wrapper script to support easily loading applications with the V4L2
# adaptation layer.

cdata = configuration_data()
cdata.set('LIBCAMERA_V4L2_SO', get_option('prefix') / libcamera_libexecdir / 'v4l2-compat.so')

configure_file(input : 'libcamerify.in',
               output : 'libcamerify',
               configuration : cdata,
               install_dir : get_option('bindir'),
               install_tag : 'bin')
