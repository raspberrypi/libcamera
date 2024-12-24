# SPDX-License-Identifier: CC0-1.0

subdir('algorithms')
subdir('data')

ipa_name = 'ipa_mali_c55'

mali_c55_ipa_sources = files([
    'ipa_context.cpp',
    'mali-c55.cpp'
])

mali_c55_ipa_sources += mali_c55_ipa_algorithms

mod = shared_module(ipa_name,
                    mali_c55_ipa_sources,
                    name_prefix : '',
                    include_directories : [ipa_includes, libipa_includes],
                    dependencies : libcamera_private,
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

ipa_names += ipa_name
