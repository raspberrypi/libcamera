ipa_vimc_sources = [
    ['ipa_vimc',         'LGPL-2.1-or-later'],
    ['ipa_vimc_isolate', 'Proprietary'],
]

foreach t : ipa_vimc_sources
    ipa = shared_module(t[0], 'vimc.cpp',
                        name_prefix : '',
                        include_directories : [ipa_includes, libipa_includes],
                        dependencies : libcamera_dep,
                        link_with : libipa,
                        install : true,
                        install_dir : ipa_install_dir,
                        cpp_args : '-DLICENSE="' + t[1] + '"')
endforeach
