# SPDX-License-Identifier: CC0-1.0

rpi_ipa_controller_sources = files([
    'algorithm.cpp',
    'controller.cpp',
    'device_status.cpp',
    'histogram.cpp',
    'rpi/af.cpp',
    'rpi/agc.cpp',
    'rpi/agc_channel.cpp',
    'rpi/alsc.cpp',
    'rpi/awb.cpp',
    'rpi/awb_bayes.cpp',
    'rpi/black_level.cpp',
    'rpi/cac.cpp',
    'rpi/ccm.cpp',
    'rpi/contrast.cpp',
    'rpi/denoise.cpp',
    'rpi/dpc.cpp',
    'rpi/geq.cpp',
    'rpi/hdr.cpp',
    'rpi/lux.cpp',
    'rpi/noise.cpp',
    'rpi/saturation.cpp',
    'rpi/sdn.cpp',
    'rpi/sharpen.cpp',
    'rpi/sync.cpp',
    'rpi/tonemap.cpp',
])

rpi_ipa_controller_deps = [
    libcamera_private,
]

if not get_option('rpi_nn_awb').disabled()
    tflite_dep = dependency('tensorflow-lite', required : false)
    if not tflite_dep.found()
        # This may not have a pkg-config file, so try and find it directly
        cc = meson.get_compiler('cpp')
        tflite_dep = cc.find_library('tensorflow-lite', required : get_option('rpi_nn_awb'))
    endif

    if tflite_dep.found()
        rpi_ipa_controller_sources += files([
            'rpi/awb_nn.cpp',
        ])
        rpi_ipa_controller_deps += tflite_dep
    endif
endif

rpi_ipa_controller_lib = static_library('rpi_ipa_controller', rpi_ipa_controller_sources,
                                        include_directories : libipa_includes,
                                        dependencies : rpi_ipa_controller_deps)
