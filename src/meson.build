# SPDX-License-Identifier: CC0-1.0

# Cache system paths
libcamera_datadir = get_option('datadir') / 'libcamera'
libcamera_libdir = get_option('libdir') / 'libcamera'
libcamera_libexecdir = get_option('libexecdir') / 'libcamera'
libcamera_sysconfdir = get_option('sysconfdir') / 'libcamera'

config_h.set('LIBCAMERA_DATA_DIR', '"' + get_option('prefix') / libcamera_datadir + '"')
config_h.set('LIBCAMERA_SYSCONF_DIR', '"' + get_option('prefix') / libcamera_sysconfdir + '"')

summary({
         'LIBCAMERA_DATA_DIR' : config_h.get('LIBCAMERA_DATA_DIR'),
         'LIBCAMERA_SYSCONF_DIR' : config_h.get('LIBCAMERA_SYSCONF_DIR'),
         }, section : 'Paths')

# Module Signing
openssl = find_program('openssl', required : false)
if openssl.found()
    ipa_priv_key = custom_target('ipa-priv-key',
                                 output : ['ipa-priv-key.pem'],
                                 command : [gen_ipa_priv_key, '@OUTPUT@'])
    config_h.set('HAVE_IPA_PUBKEY', 1)
    ipa_sign_module = true
else
    warning('openssl not found, all IPA modules will be isolated')
    ipa_sign_module = false
endif

# libyuv, used by the Android adaptation layer and the virtual pipeline handler.
# Fallback to a subproject if libyuv isn't found, as it's typically not provided
# by distributions. Where libyuv is provided by a distribution, it may not
# always supply a pkg-config implementation, requiring cxx.find_library() to
# search for it.
if not get_option('force_fallback_for').contains('libyuv')
    libyuv_dep = dependency('libyuv', required : false)
    if not libyuv_dep.found()
        libyuv_dep = cxx.find_library('yuv', has_headers : 'libyuv.h',
                                      required : false)
    endif
else
    libyuv_dep = dependency('', required : false)
endif

if (pipelines.contains('virtual') or get_option('android').allowed()) and \
   not libyuv_dep.found()
    cmake = import('cmake')

    libyuv_vars = cmake.subproject_options()
    libyuv_vars.add_cmake_defines({'CMAKE_POSITION_INDEPENDENT_CODE': 'ON'})
    libyuv_vars.set_override_option('cpp_std', 'c++17')
    libyuv_vars.append_compile_args('cpp',
         '-Wno-sign-compare',
         '-Wno-unused-variable',
         '-Wno-unused-parameter')
    libyuv_vars.append_link_args('-ljpeg')
    libyuv = cmake.subproject('libyuv', options : libyuv_vars)
    libyuv_dep = libyuv.dependency('yuv')
endif

# libcamera must be built first as a dependency to the other components.
subdir('libcamera')

subdir('android')
subdir('ipa')

subdir('apps')

subdir('gstreamer')
subdir('py')
subdir('v4l2')
