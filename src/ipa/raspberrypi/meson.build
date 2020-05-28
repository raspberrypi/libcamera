# SPDX-License-Identifier: CC0-1.0

ipa_name = 'ipa_rpi'

rpi_ipa_deps = [
    libcamera_dep,
    dependency('boost'),
    libatomic,
]

rpi_ipa_includes = [
    ipa_includes,
    libipa_includes,
    include_directories('controller')
]

rpi_ipa_sources = files([
    'raspberrypi.cpp',
    'md_parser.cpp',
    'md_parser_rpi.cpp',
    'cam_helper.cpp',
    'cam_helper_ov5647.cpp',
    'cam_helper_imx219.cpp',
    'cam_helper_imx477.cpp',
    'controller/controller.cpp',
    'controller/histogram.cpp',
    'controller/algorithm.cpp',
    'controller/rpi/alsc.cpp',
    'controller/rpi/awb.cpp',
    'controller/rpi/sharpen.cpp',
    'controller/rpi/black_level.cpp',
    'controller/rpi/focus.cpp',
    'controller/rpi/geq.cpp',
    'controller/rpi/noise.cpp',
    'controller/rpi/lux.cpp',
    'controller/rpi/agc.cpp',
    'controller/rpi/dpc.cpp',
    'controller/rpi/ccm.cpp',
    'controller/rpi/contrast.cpp',
    'controller/rpi/sdn.cpp',
    'controller/pwl.cpp',
])

mod = shared_module(ipa_name,
                    rpi_ipa_sources,
                    name_prefix : '',
                    include_directories : rpi_ipa_includes,
                    dependencies : rpi_ipa_deps,
                    link_with : libipa,
                    install : true,
                    install_dir : ipa_install_dir)

if ipa_sign_module
    custom_target(ipa_name + '.so.sign',
                  input : mod,
                  output : ipa_name + '.so.sign',
                  command : [ ipa_sign, ipa_priv_key, '@INPUT@', '@OUTPUT@' ],
                  install : false,
                  build_by_default : true)
endif

subdir('data')
