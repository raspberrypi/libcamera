# SPDX-License-Identifier: CC0-1.0

libcamera_ipa_include_dir = libcamera_include_dir / 'ipa'

libcamera_ipa_headers = files([
    'ipa_controls.h',
    'ipa_interface.h',
    'ipa_module_info.h',
])

install_headers(libcamera_ipa_headers,
                subdir: libcamera_ipa_include_dir)

libcamera_generated_ipa_headers = []

ipa_headers_install_dir = get_option('includedir') / libcamera_ipa_include_dir

#
# Prepare IPA/IPC generation components
#

core_mojom_file = 'core.mojom'
ipa_mojom_core = custom_target(core_mojom_file.split('.')[0] + '_mojom_module',
                               input : core_mojom_file,
                               output : core_mojom_file + '-module',
                               command : [
                                   mojom_parser,
                                   '--output-root', meson.project_build_root(),
                                   '--input-root', meson.project_source_root(),
                                   '--mojoms', '@INPUT@'
                               ])

# core_ipa_interface.h
libcamera_generated_ipa_headers += custom_target('core_ipa_interface_h',
                  input : ipa_mojom_core,
                  output : 'core_ipa_interface.h',
                  depends : mojom_templates,
                  install : true,
                  install_dir : ipa_headers_install_dir,
                  command : [
                      mojom_generator, 'generate',
                      '-g', 'libcamera',
                      '--bytecode_path', mojom_templates_dir,
                      '--libcamera_generate_core_header',
                      '--libcamera_output_path=@OUTPUT@',
                      './' +'@INPUT@'
                  ])

# core_ipa_serializer.h
libcamera_generated_ipa_headers += custom_target('core_ipa_serializer_h',
                  input : ipa_mojom_core,
                  output : 'core_ipa_serializer.h',
                  depends : mojom_templates,
                  command : [
                      mojom_generator, 'generate',
                      '-g', 'libcamera',
                      '--bytecode_path', mojom_templates_dir,
                      '--libcamera_generate_core_serializer',
                      '--libcamera_output_path=@OUTPUT@',
                      './' +'@INPUT@'
                  ])

ipa_mojom_files = [
    'ipu3.mojom',
    'raspberrypi.mojom',
    'rkisp1.mojom',
    'vimc.mojom',
]

ipa_mojoms = []

#
# Generate headers from templates.
#

# TODO Define per-pipeline ControlInfoMap with yaml?

foreach file : ipa_mojom_files
    name = file.split('.')[0]

    if name not in pipelines
        continue
    endif

    # {pipeline}.mojom-module
    mojom = custom_target(name + '_mojom_module',
                          input : file,
                          output : file + '-module',
                          depends : ipa_mojom_core,
                          command : [
                              mojom_parser,
                              '--output-root', meson.project_build_root(),
                              '--input-root', meson.project_source_root(),
                              '--mojoms', '@INPUT@'
                          ])

    # {pipeline}_ipa_interface.h
    header = custom_target(name + '_ipa_interface_h',
                           input : mojom,
                           output : name + '_ipa_interface.h',
                           depends : mojom_templates,
                           install : true,
                           install_dir : ipa_headers_install_dir,
                           command : [
                               mojom_generator, 'generate',
                               '-g', 'libcamera',
                               '--bytecode_path', mojom_templates_dir,
                               '--libcamera_generate_header',
                               '--libcamera_output_path=@OUTPUT@',
                               './' +'@INPUT@'
                           ])

    # {pipeline}_ipa_serializer.h
    serializer = custom_target(name + '_ipa_serializer_h',
                               input : mojom,
                               output : name + '_ipa_serializer.h',
                               depends : mojom_templates,
                               command : [
                                   mojom_generator, 'generate',
                                   '-g', 'libcamera',
                                   '--bytecode_path', mojom_templates_dir,
                                   '--libcamera_generate_serializer',
                                   '--libcamera_output_path=@OUTPUT@',
                                   './' +'@INPUT@'
                               ])

    # {pipeline}_ipa_proxy.h
    proxy_header = custom_target(name + '_proxy_h',
                                 input : mojom,
                                 output : name + '_ipa_proxy.h',
                                 depends : mojom_templates,
                                 command : [
                                     mojom_generator, 'generate',
                                     '-g', 'libcamera',
                                     '--bytecode_path', mojom_templates_dir,
                                     '--libcamera_generate_proxy_h',
                                     '--libcamera_output_path=@OUTPUT@',
                                     './' +'@INPUT@'
                                 ])

    ipa_mojoms += {
        'name': name,
        'mojom': mojom,
    }

    libcamera_generated_ipa_headers += [header, serializer, proxy_header]
endforeach

ipa_mojom_files = files(ipa_mojom_files)

# Pass this to the documentation generator in src/libcamera/ipa
ipa_mojom_files += files(['core.mojom'])
