# SPDX-License-Identifier: CC0-1.0

opt_cam = get_option('cam')
opt_lc_compliance = get_option('lc-compliance')

# libevent is needed by cam and lc-compliance. As they are both feature options,
# they can't be combined with simple boolean logic.
libevent = dependency('libevent_pthreads', required : opt_cam)
if not libevent.found()
    libevent = dependency('libevent_pthreads', required : opt_lc_compliance)
endif

libtiff = dependency('libtiff-4', required : false)

subdir('common')

subdir('lc-compliance')

subdir('cam')
subdir('qcam')
