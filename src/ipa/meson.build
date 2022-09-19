# SPDX-License-Identifier: CC0-1.0

ipa_includes = [
    libcamera_includes,
]

ipa_install_dir = libcamera_libdir
ipa_data_dir = libcamera_datadir / 'ipa'
ipa_sysconf_dir = libcamera_sysconfdir / 'ipa'

config_h.set('IPA_CONFIG_DIR',
             '"' + get_option('prefix') / ipa_sysconf_dir +
             ':' + get_option('prefix') / ipa_data_dir + '"')

config_h.set('IPA_MODULE_DIR',
             '"' + get_option('prefix') / ipa_install_dir + '"')

summary({
         'IPA_CONFIG_DIR' : config_h.get('IPA_CONFIG_DIR'),
         'IPA_MODULE_DIR' : config_h.get('IPA_MODULE_DIR'),
         }, section : 'Paths')

subdir('libipa')

ipa_sign = files('ipa-sign.sh')

ipa_names = []

ipa_modules = get_option('ipas')

# Tests require the vimc IPA, similar to vimc pipline-handler for their
# execution. Include it automatically when tests are enabled.
if get_option('test') and 'vimc' not in ipa_modules
    message('Enabling vimc IPA to support tests')
    ipa_modules += ['vimc']
endif

enabled_ipa_modules = []

# The ipa-sign-install.sh script which uses the ipa_names variable will itself
# prepend MESON_INSTALL_DESTDIR_PREFIX to each ipa module name, therefore we
# must not include the prefix string here.
foreach pipeline : pipelines
    if ipa_modules.contains(pipeline)
        subdir(pipeline)
        ipa_names += ipa_install_dir / ipa_name + '.so'
        enabled_ipa_modules += pipeline
    endif
endforeach

if ipa_sign_module
    # Regenerate the signatures for all IPA modules. We can't simply install the
    # .sign files, as meson strips the DT_RPATH and DT_RUNPATH from binaries at
    # install time, which invalidates the signatures.
    meson.add_install_script('ipa-sign-install.sh',
                             ipa_priv_key.full_path(),
                             ipa_names)
endif
