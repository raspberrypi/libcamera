# SPDX-License-Identifier: CC0-1.0

if get_option('android')
    subdir('android')
endif

openssl = find_program('openssl', required : true)
if openssl.found()
    ipa_gen_priv_key = files('ipa/gen-ipa-priv-key.sh')
    ipa_priv_key = custom_target('ipa-priv-key',
                                 output : [ 'ipa-priv-key.pem' ],
                                 command : [ ipa_gen_priv_key, '@OUTPUT@' ])
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
