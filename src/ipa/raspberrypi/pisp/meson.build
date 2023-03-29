# SPDX-License-Identifier: CC0-1.0

ipa_name = 'ipa_pisp'

pisp_ipa_deps = [
    libcamera_private,
    libatomic,
    libpisp_dep,
]

pisp_ipa_includes = [
    ipa_includes,
    libipa_includes,
]

pisp_ipa_sources = files([
    'pisp.cpp',
])

rpi_ipa_common_sources = files([
    '../common/cam_helper.cpp',
    '../common/cam_helper_ov5647.cpp',
    '../common/cam_helper_imx219.cpp',
    '../common/cam_helper_imx296.cpp',
    '../common/cam_helper_imx477.cpp',
    '../common/cam_helper_imx708.cpp',
    '../common/ipa_base.cpp',
    '../common/controller/algorithm.cpp',
    '../common/controller/controller.cpp',
    '../common/controller/device_status.cpp',
    '../common/controller/histogram.cpp',
    '../common/controller/pwl.cpp',
    '../common/controller/rpi/af.cpp',
    '../common/controller/rpi/agc.cpp',
    '../common/controller/rpi/alsc.cpp',
    '../common/controller/rpi/awb.cpp',
    '../common/controller/rpi/ccm.cpp',
    '../common/controller/rpi/contrast.cpp',
    '../common/controller/rpi/sharpen.cpp',
    '../common/controller/rpi/black_level.cpp',
    '../common/controller/rpi/denoise.cpp',
    '../common/controller/rpi/dpc.cpp',
    '../common/controller/rpi/geq.cpp',
    '../common/controller/rpi/hdr.cpp',
    '../common/controller/rpi/lux.cpp',
    '../common/controller/rpi/noise.cpp',
    '../common/controller/rpi/saturation.cpp',
    '../common/controller/rpi/sdn.cpp',
    '../common/controller/rpi/tonemap.cpp',
    '../common/md_parser_smia.cpp',
])

rpi_ipa_common_includes = [
    include_directories('../common/'),
    include_directories('../common/controller'),
]

pisp_ipa_includes += rpi_ipa_common_includes
pisp_ipa_sources += rpi_ipa_common_sources

mod = shared_module(ipa_name,
                    [pisp_ipa_sources, libcamera_generated_ipa_headers],
                    name_prefix : '',
                    include_directories : pisp_ipa_includes,
                    dependencies : pisp_ipa_deps,
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
