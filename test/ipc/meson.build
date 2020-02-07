# SPDX-License-Identifier: CC0-1.0

ipc_tests = [
    [ 'unixsocket',  'unixsocket.cpp' ],
]

foreach t : ipc_tests
    exe = executable(t[0], t[1],
                     dependencies : libcamera_dep,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(t[0], exe, suite : 'ipc')
endforeach
