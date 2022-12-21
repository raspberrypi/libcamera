# SPDX-License-Identifier: CC0-1.0

if not is_variable('v4l2_compat')
    subdir_done()
endif

# If ASan is enabled but the ASan runtime shared library is missing,
# v4l2_compat_test.py won't be able to LD_PRELOAD it, resulting in a link order
# runtime check failure as v4l2-ctl and v4l2-compliance are not linked to ASan.
# Skip the test in that case.

if asan_runtime_missing
    warning('Unable to get path to ASan runtime, v4l2_compat test disabled')
    subdir_done()
endif

v4l2_compat_test = files('v4l2_compat_test.py')
v4l2_compat_args = []

if asan_enabled
    v4l2_compat_args += ['-s', asan_runtime]
endif

v4l2_compat_args += [v4l2_compat]

test('v4l2_compat_test', v4l2_compat_test,
     args : v4l2_compat_args,
     suite : 'v4l2_compat',
     timeout : 60)
