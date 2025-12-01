# SPDX-License-Identifier: CC0-1.0
# Copyright (C) 2023, Raspberry Pi Ltd

frontend_sources = files([
    'frontend.cpp'
])

frontend_headers = files([
    'frontend.hpp',
    'pisp_fe_config.h',
    'pisp_statistics.h'
])

frontend_include_dir = pisp_include_dir / 'frontend'
install_headers(frontend_headers, subdir: frontend_include_dir)
