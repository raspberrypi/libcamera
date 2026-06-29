# SPDX-License-Identifier: CC0-1.0

apps_sources = files([
    'image.cpp',
    'options.cpp',
    'ppm_writer.cpp',
    'stream_options.cpp',
])

apps_cpp_args = []

# gcc's optimiser is prone to false-positive diagnostics on this std::string-heavy
# app code (e.g. -Wrestrict firing on `s = "--"; s += ptr;` at -O3). The project
# builds with werror=true, so a single such false positive aborts the whole build
# and bricks an install. Demote the notorious offenders to non-fatal for the apps
# only — core libcamera stays strict. get_supported_arguments() keeps this safe
# across compiler versions (flags the compiler doesn't know are dropped).
apps_cxx = meson.get_compiler('cpp')
if apps_cxx.get_id() == 'gcc'
    apps_cpp_args += apps_cxx.get_supported_arguments([
        '-Wno-error=restrict',
        '-Wno-error=stringop-overflow',
        '-Wno-error=stringop-overread',
        '-Wno-error=array-bounds',
        '-Wno-error=dangling-reference',
    ])
endif

if libevent.found()
    apps_sources += files([
        'event_loop.cpp',
    ])
endif

if libtiff.found()
    apps_cpp_args += ['-DHAVE_TIFF']
    apps_sources += files([
        'dng_writer.cpp',
    ])
endif

apps_lib = static_library('apps', apps_sources,
                          cpp_args : apps_cpp_args,
                          dependencies : [libcamera_public])
