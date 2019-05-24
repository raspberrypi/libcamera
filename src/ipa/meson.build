ipa_dummy_sources = files([
    'ipa_dummy.cpp',
])

ipa_install_dir = join_paths(get_option('libdir'), 'libcamera')

ipa_dummy = shared_library('ipa_dummy',
                           ipa_dummy_sources,
                           name_prefix : '',
                           include_directories : libcamera_includes,
                           install : true,
                           install_dir : ipa_install_dir)

config_h.set('IPA_MODULE_DIR',
             '"' + join_paths(get_option('prefix'), ipa_install_dir) + '"')
