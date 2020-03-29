if get_option('android')
    subdir('android')
endif

ipa_gen_priv_key = find_program('ipa/gen-ipa-priv-key.sh')
ipa_priv_key = custom_target('ipa-priv-key',
                             output : [ 'ipa-priv-key.pem' ],
                             command : [ ipa_gen_priv_key, '@OUTPUT@' ])

subdir('libcamera')
subdir('ipa')
subdir('cam')
subdir('qcam')

if get_option('v4l2')
    subdir('v4l2')
endif

subdir('gstreamer')
