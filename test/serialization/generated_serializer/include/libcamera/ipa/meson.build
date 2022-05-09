# SPDX-License-Identifier: CC0-1.0

# test.mojom-module
mojom = custom_target('test_mojom_module',
                      input : 'test.mojom',
                      output : 'test.mojom-module',
                      command : [
                          mojom_parser,
                          '--output-root', meson.project_build_root(),
                          '--input-root', meson.project_source_root(),
                          '--mojoms', '@INPUT@'
                      ])

# test_ipa_interface.h
generated_test_header = custom_target('test_ipa_interface_h',
                       input : mojom,
                       output : 'test_ipa_interface.h',
                       depends : mojom_templates,
                       command : [
                           mojom_generator, 'generate',
                           '-g', 'libcamera',
                           '--bytecode_path', mojom_templates_dir,
                           '--libcamera_generate_header',
                           '--libcamera_output_path=@OUTPUT@',
                           './' +'@INPUT@'
                       ])

# test_ipa_serializer.h
generated_test_serializer = custom_target('test_ipa_serializer_h',
                           input : mojom,
                           output : 'test_ipa_serializer.h',
                           depends : mojom_templates,
                           command : [
                               mojom_generator, 'generate',
                               '-g', 'libcamera',
                               '--bytecode_path', mojom_templates_dir,
                               '--libcamera_generate_serializer',
                               '--libcamera_output_path=@OUTPUT@',
                               './' +'@INPUT@'
                           ])
