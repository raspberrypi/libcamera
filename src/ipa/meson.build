ipa_install_dir = join_paths(get_option('libdir'), 'libcamera')

ipa_includes = [
    libcamera_includes,
    libcamera_internal_includes,
]

config_h.set('IPA_MODULE_DIR',
             '"' + join_paths(get_option('prefix'), ipa_install_dir) + '"')

subdir('libipa')

ipas = ['rkisp1', 'vimc']

foreach pipeline : get_option('pipelines')
    if ipas.contains(pipeline)
        subdir(pipeline)
    endif
endforeach
