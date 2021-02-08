# SPDX-License-Identifier: CC0-1.0

ipa_name = 'ipa_ipu3'

mod = shared_module(ipa_name,
                    'ipu3.cpp',
                    name_prefix : '',
                    include_directories : [ipa_includes, libipa_includes],
                    dependencies : [libatomic, libcamera_dep],
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
