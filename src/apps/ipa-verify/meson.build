# SPDX-License-Identifier: CC0-1.0

if not ipa_sign_module
    subdir_done()
endif

ipa_verify_sources = files([
    'main.cpp',
])

ipa_verify  = executable('ipa_verify', ipa_verify_sources,
                         dependencies : [
                             libcamera_private,
                         ],
                         install : false)
