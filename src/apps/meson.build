# SPDX-License-Identifier: CC0-1.0

opt_cam = get_option('cam')
opt_lc_compliance = get_option('lc-compliance')

# libevent is needed by cam and lc-compliance. As they are both feature options,
# they can't be combined with simple boolean logic.
libevent = dependency('libevent_pthreads', required : opt_cam)
if not libevent.found()
    libevent = dependency('libevent_pthreads', required : opt_lc_compliance)
endif

libtiff = dependency('libtiff-4', required : get_option('apps-output-dng'))

subdir('common')

subdir('lc-compliance')

subdir('cam')
subdir('qcam')

subdir('ipa-verify')

summary({
            'cam application': cam_enabled,
            'cam options': cam_options,
            'DNG output support': libtiff.found(),
            'qcam application': qcam_enabled,
            'lc-compliance application': lc_compliance_enabled,
        },
        section : 'Applications',
        bool_yn : true)
