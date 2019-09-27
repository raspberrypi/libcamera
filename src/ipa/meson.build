ipa_vimc_sources = [
    ['ipa_vimc',         'LGPL-2.1-or-later'],
    ['ipa_vimc_isolate', 'Proprietary'],
]

ipa_install_dir = join_paths(get_option('libdir'), 'libcamera')

ipa_includes = [
    libcamera_includes,
    libcamera_internal_includes,
]

foreach t : ipa_vimc_sources
    ipa = shared_module(t[0], 'ipa_vimc.cpp',
                        name_prefix : '',
                        include_directories : ipa_includes,
                        dependencies : libcamera_dep,
                        install : true,
                        install_dir : ipa_install_dir,
                        cpp_args : '-DLICENSE="' + t[1] + '"')
endforeach

config_h.set('IPA_MODULE_DIR',
             '"' + join_paths(get_option('prefix'), ipa_install_dir) + '"')

subdir('rkisp1')
