ipa_dummy_sources = [
    ['ipa_dummy', 'ipa_dummy.cpp'],
    ['ipa_dummy_isolate', 'ipa_dummy_isolate.cpp'],
]

ipa_install_dir = join_paths(get_option('libdir'), 'libcamera')

foreach t : ipa_dummy_sources
    ipa = shared_module(t[0],
                        t[1],
                        name_prefix : '',
                        include_directories : libcamera_includes,
                        install : true,
                        install_dir : ipa_install_dir)
endforeach

config_h.set('IPA_MODULE_DIR',
             '"' + join_paths(get_option('prefix'), ipa_install_dir) + '"')
