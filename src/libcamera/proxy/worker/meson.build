# SPDX-License-Identifier: CC0-1.0

ipa_proxy_sources = [
    ['ipa_proxy_linux', 'ipa_proxy_linux_worker.cpp']
]

proxy_install_dir = join_paths(get_option('libexecdir'), 'libcamera')

foreach t : ipa_proxy_sources
    proxy = executable(t[0], t[1],
                       install : true,
                       install_dir : proxy_install_dir,
                       dependencies : libcamera_dep)
endforeach

config_h.set('IPA_PROXY_DIR',
             '"' + join_paths(get_option('prefix'), proxy_install_dir) + '"')
