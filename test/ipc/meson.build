# SPDX-License-Identifier: CC0-1.0

ipc_tests = [
    ['unixsocket_ipc', 'unixsocket_ipc.cpp'],
    ['unixsocket',     'unixsocket.cpp'],
]

foreach test : ipc_tests
    exe = executable(test[0], test[1],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test[0], exe, suite : 'ipc')
endforeach
