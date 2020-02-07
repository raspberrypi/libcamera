# SPDX-License-Identifier: CC0-1.0

libipa_headers = files([
    'ipa_interface_wrapper.h',
])

libipa_sources = files([
    'ipa_interface_wrapper.cpp',
])

libipa_includes = include_directories('..')

libipa = static_library('ipa', libipa_sources,
                        include_directories : ipa_includes,
                        dependencies : libcamera_dep)
