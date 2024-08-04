# SPDX-License-Identifier: CC0-1.0

ipa_name = 'ipa_rpi_vc4'

vc4_ipa_deps = [
    libcamera_private,
    libatomic,
]

vc4_ipa_libs = [
    rpi_ipa_cam_helper_lib,
    rpi_ipa_common_lib,
    rpi_ipa_controller_lib
]

vc4_ipa_includes = [
    ipa_includes,
]

vc4_ipa_sources = files([
    'vc4.cpp',
])

vc4_ipa_includes += include_directories('..')

mod = shared_module(ipa_name, vc4_ipa_sources,
                    name_prefix : '',
                    include_directories : vc4_ipa_includes,
                    dependencies : [vc4_ipa_deps, libipa_dep],
                    link_whole : vc4_ipa_libs,
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
