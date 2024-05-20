# SPDX-License-Identifier: CC0-1.0

ipa_name = 'ipa_soft_simple'

soft_simple_sources = files([
    'soft_simple.cpp',
    'black_level.cpp',
])

mod = shared_module(ipa_name,
                    [soft_simple_sources, libcamera_generated_ipa_headers],
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

subdir('data')

ipa_names += ipa_name
