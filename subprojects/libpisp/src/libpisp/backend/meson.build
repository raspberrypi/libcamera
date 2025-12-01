# SPDX-License-Identifier: CC0-1.0
# Copyright (C) 2023, Raspberry Pi Ltd

config_install_dir = get_option('prefix') / get_option('datadir') / 'libpisp'

conf_data = configuration_data()
conf_data.set('PISP_BE_CONFIG_DIR', '"' + config_install_dir + '"')
pisp_build_config = configure_file(output : 'pisp_build_config.h', configuration : conf_data)

backend_sources = files([
    'backend.cpp',
    'backend_debug.cpp',
    'backend_default_config.cpp',
    'backend_prepare.cpp',
])

backend_headers = files([
    'backend.hpp',
    'pisp_be_config.h',
])

backend_include_dir = pisp_include_dir / 'backend'
install_headers(backend_headers, subdir: backend_include_dir)

install_data('backend_default_config.json',
             install_dir : config_install_dir)

# Copy the json config file to the build directory for running locally.
custom_target('Default config to build dir',
              input : 'backend_default_config.json',
              output : 'backend_default_config.json',
              command : ['cp', '@INPUT@', '@OUTPUT@'],
              install : false,
              build_by_default : true)

subdir('tiling')
