# SPDX-License-Identifier: CC0-1.0

ipa_proxy_sources = [
    ['ipa_proxy_linux', 'ipa_proxy_linux_worker.cpp']
]

proxy_install_dir = join_paths(get_option('libexecdir'), 'libcamera')

# generate {pipeline}_ipa_proxy_worker.cpp
foreach mojom : ipa_mojoms
    worker = custom_target(mojom['name'] + '_proxy_worker',
                           input : mojom['mojom'],
                           output : mojom['name'] + '_ipa_proxy_worker.cpp',
                           depends : mojom_templates,
                           command : [
                               mojom_generator, 'generate',
                               '-g', 'libcamera',
                               '--bytecode_path', mojom_templates_dir,
                               '--libcamera_generate_proxy_worker',
                               '--libcamera_output_path=@OUTPUT@',
                               './' + '@INPUT@'
                           ])

    proxy = executable(mojom['name'] + '_ipa_proxy',
                       [worker, libcamera_generated_ipa_headers],
                       install : true,
                       install_dir : proxy_install_dir,
                       dependencies : libcamera_dep)
endforeach

config_h.set('IPA_PROXY_DIR',
             '"' + join_paths(get_option('prefix'), proxy_install_dir) + '"')
