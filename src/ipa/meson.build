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

ipa_sign = files('ipa-sign.sh')

ipas = ['raspberrypi', 'rkisp1', 'vimc']

foreach pipeline : get_option('pipelines')
    if ipas.contains(pipeline)
        subdir(pipeline)
    endif
endforeach

if ipa_sign_module
    # Regenerate the signatures for all IPA modules. We can't simply install the
    # .sign files, as meson strips the DT_RPATH and DT_RUNPATH from binaries at
    # install time, which invalidates the signatures.
    meson.add_install_script('ipa-sign-install.sh',
                             ipa_install_dir,
                             ipa_priv_key.full_path())
endif
