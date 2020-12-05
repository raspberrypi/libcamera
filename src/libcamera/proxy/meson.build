# SPDX-License-Identifier: CC0-1.0

# generate {pipeline}_ipa_proxy.cpp
foreach mojom : ipa_mojoms
    proxy = custom_target(mojom['name'] + '_proxy_cpp',
                          input : mojom['mojom'],
                          output : mojom['name'] + '_ipa_proxy.cpp',
                          depends : mojom_templates,
                          command : [
                              mojom_generator, 'generate',
                              '-g', 'libcamera',
                              '--bytecode_path', mojom_templates_dir,
                              '--libcamera_generate_proxy_cpp',
                              '--libcamera_output_path=@OUTPUT@',
                              './' + '@INPUT@'
                          ])

    libcamera_sources += proxy
endforeach
