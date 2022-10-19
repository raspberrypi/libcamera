# SPDX-License-Identifier: CC0-1.0

# Cache system paths
libcamera_datadir = get_option('datadir') / 'libcamera'
libcamera_libdir = get_option('libdir') / 'libcamera'
libcamera_sysconfdir = get_option('sysconfdir') / 'libcamera'

config_h.set('LIBCAMERA_DATA_DIR', '"' + get_option('prefix') / libcamera_datadir + '"')
config_h.set('LIBCAMERA_SYSCONF_DIR', '"' + get_option('prefix') / libcamera_sysconfdir + '"')

summary({
         'LIBCAMERA_DATA_DIR' : config_h.get('LIBCAMERA_DATA_DIR'),
         'LIBCAMERA_SYSCONF_DIR' : config_h.get('LIBCAMERA_SYSCONF_DIR'),
         }, section : 'Paths')

# Module Signing
openssl = find_program('openssl', required : false)
if openssl.found()
    ipa_priv_key = custom_target('ipa-priv-key',
                                 output : ['ipa-priv-key.pem'],
                                 command : [gen_ipa_priv_key, '@OUTPUT@'])
    config_h.set('HAVE_IPA_PUBKEY', 1)
    ipa_sign_module = true
else
    warning('openssl not found, all IPA modules will be isolated')
    ipa_sign_module = false
endif

# libcamera must be built first as a dependency to the other components.
subdir('libcamera')

subdir('android')
subdir('ipa')

subdir('apps')

subdir('gstreamer')
subdir('py')
subdir('v4l2')
