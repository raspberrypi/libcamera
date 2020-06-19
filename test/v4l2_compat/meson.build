# SPDX-License-Identifier: CC0-1.0

if is_variable('v4l2_compat')
    v4l2_compat_test = files('v4l2_compat_test.py')

    test('v4l2_compat_test', v4l2_compat_test,
         args : v4l2_compat,
         suite : 'v4l2_compat',
         timeout : 60)
endif
