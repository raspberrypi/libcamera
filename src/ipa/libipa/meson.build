# SPDX-License-Identifier: CC0-1.0

libipa_headers = files([
    'agc.h',
    'agc_mean_luminance.h',
    'agc_msv.h',
    'algorithm.h',
    'awb_bayes.h',
    'awb_grey.h',
    'awb.h',
    'camera_sensor_helper.h',
    'ccm.h',
    'colours.h',
    'exposure_mode_helper.h',
    'fc_queue.h',
    'fixedpoint.h',
    'gamma.h',
    'histogram.h',
    'interpolator.h',
    'lsc.h',
    'lsc_base.h',
    'lsc_polynomial.h',
    'lsc_table.h',
    'lux.h',
    'module.h',
    'pwl.h',
    'quantized.h',
    'v4l2_params.h',
    'v4l2_stats.h',
])

libipa_sources = files([
    'agc.cpp',
    'agc_mean_luminance.cpp',
    'agc_msv.cpp',
    'algorithm.cpp',
    'awb_bayes.cpp',
    'awb_grey.cpp',
    'awb.cpp',
    'camera_sensor_helper.cpp',
    'ccm.cpp',
    'colours.cpp',
    'exposure_mode_helper.cpp',
    'fc_queue.cpp',
    'fixedpoint.cpp',
    'gamma.cpp',
    'histogram.cpp',
    'interpolator.cpp',
    'lsc.cpp',
    'lsc_base.cpp',
    'lsc_polynomial.cpp',
    'lsc_table.cpp',
    'lux.cpp',
    'module.cpp',
    'pwl.cpp',
    'quantized.cpp',
    'v4l2_params.cpp',
    'v4l2_stats.cpp',
])

libipa_includes = include_directories('..')

libipa = static_library('ipa', [libipa_sources, libipa_headers],
                        include_directories : ipa_includes,
                        dependencies : libcamera_private)

libipa_dep = declare_dependency(sources : libipa_headers,
                                include_directories : libipa_includes,
                                link_with : libipa)
