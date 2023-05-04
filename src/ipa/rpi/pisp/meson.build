# SPDX-License-Identifier: CC0-1.0

ipa_name = 'ipa_rpi_pisp'

pisp_ipa_deps = [
    libcamera_private,
    libatomic,
    libpisp_dep,
]

pisp_ipa_libs = [
    rpi_ipa_cam_helper_lib,
    rpi_ipa_common_lib,
    rpi_ipa_controller_lib
]

pisp_ipa_includes = [
    ipa_includes,
    libipa_includes,
]

pisp_ipa_sources = files([
    'pisp.cpp',
])

pisp_ipa_includes += include_directories('..')

mod = shared_module(ipa_name, pisp_ipa_sources,
                    name_prefix : '',
                    include_directories : pisp_ipa_includes,
                    dependencies : pisp_ipa_deps,
                    link_with : libipa,
                    link_whole : pisp_ipa_libs,
                    install : true,
                    cpp_args : '-Wno-address-of-packed-member',
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
