# SPDX-License-Identifier: CC0-1.0

libcamera_ipa_interfaces = []

foreach file : ipa_mojom_files
    # {pipeline}_ipa_interface.cpp
    libcamera_ipa_interfaces += \
        custom_target(input : file,
                      output : '@BASENAME@_ipa_interface.cpp',
                      command : [
                          mojom_docs_extractor,
                          '-o', '@OUTPUT@', '@INPUT@'
                      ])
endforeach
