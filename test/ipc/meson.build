# SPDX-License-Identifier: CC0-1.0

ipc_tests = [
    {'name': 'unixsocket_ipc', 'sources': ['unixsocket_ipc.cpp']},
    {'name': 'unixsocket', 'sources': ['unixsocket.cpp']},
]

foreach test : ipc_tests
    exe = executable(test['name'], test['sources'],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test['name'], exe, suite : 'ipc')
endforeach
