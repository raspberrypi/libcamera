# SPDX-License-Identifier: CC0-1.0

subdir('android')

openssl = find_program('openssl', required : true)
if openssl.found()
    ipa_priv_key = custom_target('ipa-priv-key',
                                 output : [ 'ipa-priv-key.pem' ],
                                 command : [ gen_ipa_priv_key, '@OUTPUT@' ])
    config_h.set('HAVE_IPA_PUBKEY', 1)
    ipa_sign_module = true
else
    ipa_sign_module = false
endif

subdir('libcamera')
subdir('ipa')
subdir('cam')
subdir('qcam')

if get_option('v4l2')
    subdir('v4l2')
endif

subdir('gstreamer')
