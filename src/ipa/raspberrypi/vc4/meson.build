# SPDX-License-Identifier: CC0-1.0

ipa_name = 'ipa_vc4'

vc4_ipa_deps = [
    libcamera_private,
    libatomic,
]

vc4_ipa_includes = [
    ipa_includes,
    libipa_includes,
]

vc4_ipa_sources = files([
    'vc4.cpp',
])

rpi_ipa_common_sources = files([
    '../common/cam_helper.cpp',
    '../common/cam_helper_ov5647.cpp',
    '../common/cam_helper_imx219.cpp',
    '../common/cam_helper_imx290.cpp',
    '../common/cam_helper_imx296.cpp',
    '../common/cam_helper_imx477.cpp',
    '../common/cam_helper_imx519.cpp',
    '../common/cam_helper_imx708.cpp',
    '../common/cam_helper_ov9281.cpp',
    '../common/ipa_base.cpp',
    '../common/controller/controller.cpp',
    '../common/controller/histogram.cpp',
    '../common/controller/algorithm.cpp',
    '../common/controller/rpi/af.cpp',
    '../common/controller/rpi/alsc.cpp',
    '../common/controller/rpi/awb.cpp',
    '../common/controller/rpi/sharpen.cpp',
    '../common/controller/rpi/black_level.cpp',
    '../common/controller/rpi/geq.cpp',
    '../common/controller/rpi/noise.cpp',
    '../common/controller/rpi/lux.cpp',
    '../common/controller/rpi/agc.cpp',
    '../common/controller/rpi/dpc.cpp',
    '../common/controller/rpi/ccm.cpp',
    '../common/controller/rpi/contrast.cpp',
    '../common/controller/rpi/sdn.cpp',
    '../common/controller/pwl.cpp',
    '../common/controller/device_status.cpp',
    '../common/md_parser_smia.cpp',
])

rpi_ipa_common_includes = [
    include_directories('../common/'),
    include_directories('../common/controller'),
]

vc4_ipa_includes += rpi_ipa_common_includes
vc4_ipa_sources += rpi_ipa_common_sources

mod = shared_module(ipa_name,
                    [vc4_ipa_sources, libcamera_generated_ipa_headers],
                    name_prefix : '',
                    include_directories : vc4_ipa_includes,
                    dependencies : vc4_ipa_deps,
                    link_with : libipa,
                    install : true,
                    install_dir : ipa_install_dir)

if ipa_sign_module
    custom_target(ipa_name + '.so.sign',
                  input : mod,
                  output : ipa_name + '.so.sign',
                  command : [ipa_sign, ipa_priv_key, '@INPUT@', '@OUTPUT@'],
                  install : false,
                  build_by_default : true)
endif

subdir('data')
