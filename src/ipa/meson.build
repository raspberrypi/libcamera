ipa_dummy_sources = [
    ['ipa_dummy',         'LGPL-2.1-or-later'],
    ['ipa_dummy_isolate', 'Proprietary'],
]

ipa_install_dir = join_paths(get_option('libdir'), 'libcamera')

foreach t : ipa_dummy_sources
    ipa = shared_module(t[0], 'ipa_dummy.cpp',
                        name_prefix : '',
                        include_directories : libcamera_includes,
                        install : true,
                        install_dir : ipa_install_dir,
                        cpp_args : '-DLICENSE="' + t[1] + '"')
endforeach

config_h.set('IPA_MODULE_DIR',
             '"' + join_paths(get_option('prefix'), ipa_install_dir) + '"')
