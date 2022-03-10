# SPDX-License-Identifier: CC0-1.0

ipa_name = 'ipa_pisp'

pisp_ipa_deps = [
    libcamera_private,
    dependency('boost'),
    libatomic,
]

pisp_ipa_includes = [
    rpi_ipa_common_includes,
    include_directories('controller')
]

pisp_ipa_sources = files([
    'pisp.cpp',
    'controller/rpi/agc.cpp',
    'controller/rpi/awb.cpp',
])

pisp_ipa_sources += rpi_ipa_common_sources
pisp_ipa_sources += frontend_sources
pisp_ipa_sources += backend_sources
pisp_ipa_includes += [frontend_include_dir, backend_include_dir]

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
