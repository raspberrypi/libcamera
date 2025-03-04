# SPDX-License-Identifier: CC0-1.0

libipa_headers = files([
    'agc_mean_luminance.h',
    'algorithm.h',
    'awb_bayes.h',
    'awb_grey.h',
    'awb.h',
    'camera_sensor_helper.h',
    'colours.h',
    'exposure_mode_helper.h',
    'fc_queue.h',
    'fixedpoint.h',
    'histogram.h',
    'interpolator.h',
    'lsc_polynomial.h',
    'lux.h',
    'module.h',
    'pwl.h',
])

libipa_sources = files([
    'agc_mean_luminance.cpp',
    'algorithm.cpp',
    'awb_bayes.cpp',
    'awb_grey.cpp',
    'awb.cpp',
    'camera_sensor_helper.cpp',
    'colours.cpp',
    'exposure_mode_helper.cpp',
    'fc_queue.cpp',
    'fixedpoint.cpp',
    'histogram.cpp',
    'interpolator.cpp',
    'lsc_polynomial.cpp',
    'lux.cpp',
    'module.cpp',
    'pwl.cpp',
])

libipa_includes = include_directories('..')

libipa = static_library('ipa', [libipa_sources, libipa_headers],
                        include_directories : ipa_includes,
                        dependencies : libcamera_private)

libipa_dep = declare_dependency(sources : libipa_headers,
                                include_directories : libipa_includes,
                                link_with : libipa)
