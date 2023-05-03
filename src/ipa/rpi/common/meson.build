# SPDX-License-Identifier: CC0-1.0

rpi_ipa_common_sources = files([
    'ipa_base.cpp',
])

rpi_ipa_common_includes = [
    include_directories('..'),
]

rpi_ipa_common_deps = [
    libcamera_private,
]

rpi_ipa_common_lib = static_library('rpi_ipa_common', rpi_ipa_common_sources,
                                    include_directories : rpi_ipa_common_includes,
                                    dependencies : rpi_ipa_common_deps)
