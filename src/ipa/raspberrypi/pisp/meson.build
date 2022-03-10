# SPDX-License-Identifier: CC0-1.0

ipa_name = 'ipa_pisp'

pisp_ipa_deps = [
    libcamera_private,
    dependency('boost'),
    libatomic,
]

pisp_ipa_includes = [
    rpi_ipa_common_includes,
]

pisp_ipa_sources = files([
    'pisp.cpp',
])

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

