ipa_install_dir = join_paths(get_option('libdir'), 'libcamera')

ipa_includes = [
    libcamera_includes,
    libcamera_internal_includes,
]

config_h.set('IPA_MODULE_DIR',
             '"' + join_paths(get_option('prefix'), ipa_install_dir) + '"')

subdir('libipa')
subdir('rkisp1')
subdir('vimc')
