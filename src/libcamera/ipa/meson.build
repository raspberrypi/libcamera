# SPDX-License-Identifier: CC0-1.0

libcamera_ipa_interfaces = []

foreach file : ipa_mojom_files
    name = '@0@'.format(file).split('/')[-1].split('.')[0]

    # {pipeline}_ipa_interface.cpp
    libcamera_ipa_interfaces += \
        custom_target(name + '_ipa_interface_cpp',
                      input : file,
                      output : name + '_ipa_interface.cpp',
                      command : [
                          mojom_docs_extractor,
                          '-o', '@OUTPUT@', '@INPUT@'
                      ])
endforeach
