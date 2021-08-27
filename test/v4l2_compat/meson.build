# SPDX-License-Identifier: CC0-1.0

# If ASan is enabled, the link order runtime check will fail as v4l2-ctl and
# v4l2-compliance are not linked to ASan. Skip the test in that case.
#
# TODO: Find a way to LD_PRELOAD the ASan dynamic library instead, in a
# cross-platform way with support for both gcc and clang.

if get_option('b_sanitize').contains('address')
    subdir_done()
endif

if is_variable('v4l2_compat')
    v4l2_compat_test = files('v4l2_compat_test.py')

    test('v4l2_compat_test', v4l2_compat_test,
         args : v4l2_compat,
         suite : 'v4l2_compat',
         timeout : 60)
endif
