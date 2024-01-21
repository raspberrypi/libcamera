# SPDX-License-Identifier: CC0-1.0

rpi_ipa_cam_helper_sources = files([
    'cam_helper.cpp',
    'cam_helper_ov5647.cpp',
    'cam_helper_imx219.cpp',
    'cam_helper_imx283.cpp',
    'cam_helper_imx294.cpp',
    'cam_helper_imx290.cpp',
    'cam_helper_imx296.cpp',
    'cam_helper_imx477.cpp',
    'cam_helper_imx519.cpp',
    'cam_helper_imx585.cpp',
    'cam_helper_imx708.cpp',
    'cam_helper_ov9281.cpp',
    'md_parser_smia.cpp',
])

rpi_ipa_cam_helper_includes = [
    include_directories('..'),
]

rpi_ipa_cam_helper_deps = [
    libcamera_private,
]

rpi_ipa_cam_helper_lib = static_library('rpi_ipa_cam_helper', rpi_ipa_cam_helper_sources,
                                        include_directories : rpi_ipa_cam_helper_includes,
                                        dependencies : rpi_ipa_cam_helper_deps)
