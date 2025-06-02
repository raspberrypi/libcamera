# SPDX-License-Identifier: CC0-1.0

ipa_includes = [
    libcamera_includes,
]

ipa_install_dir = libcamera_libdir / 'ipa'
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
enabled_ipa_names = []
ipa_names = []

subdirs = []
foreach pipeline : pipelines
    # The current implementation expects the IPA module name to match the
    # pipeline name.
    # \todo Make the IPA naming scheme more flexible.
    if not ipa_modules.contains(pipeline)
        continue
    endif
    enabled_ipa_names += pipeline

    # Allow multi-level directory structuring for the IPAs if needed.
    pipeline = pipeline.split('/')[0]
    if pipeline in subdirs
        continue
    endif

    subdirs += pipeline
    subdir(pipeline)

    # Don't reuse the pipeline variable below, the subdirectory may have
    # overwritten it.
endforeach

# The ipa-sign-install.sh script which uses the enabled_ipa_modules variable
# will itself prepend MESON_INSTALL_DESTDIR_PREFIX to each ipa module name,
# therefore we must not include the prefix string here.
foreach ipa_name : ipa_names
    enabled_ipa_modules += ipa_install_dir / ipa_name + '.so'
endforeach

if ipa_sign_module
    # Regenerate the signatures for all IPA modules. We can't simply install the
    # .sign files, as meson strips the DT_RPATH and DT_RUNPATH from binaries at
    # install time, which invalidates the signatures.
    meson.add_install_script('ipa-sign-install.sh',
                             ipa_priv_key.full_path(),
                             enabled_ipa_modules,
                             install_tag : 'runtime')
endif
