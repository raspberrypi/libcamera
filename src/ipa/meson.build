ipa_install_dir = join_paths(get_option('libdir'), 'libcamera')
ipa_data_dir = join_paths(get_option('datadir'), 'libcamera', 'ipa')
ipa_sysconf_dir = join_paths(get_option('sysconfdir'), 'libcamera', 'ipa')

ipa_includes = [
    libcamera_includes,
    libcamera_internal_includes,
]

config_h.set('IPA_CONFIG_DIR',
             '"' + join_paths(get_option('prefix'), ipa_sysconf_dir) +
             ':' + join_paths(get_option('prefix'), ipa_data_dir) + '"')

config_h.set('IPA_MODULE_DIR',
             '"' + join_paths(get_option('prefix'), ipa_install_dir) + '"')

subdir('libipa')

ipa_sign = find_program('ipa-sign.sh')

ipas = ['rkisp1', 'vimc']

foreach pipeline : get_option('pipelines')
    if ipas.contains(pipeline)
        subdir(pipeline)
    endif
endforeach
