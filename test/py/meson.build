# SPDX-License-Identifier: CC0-1.0

if not pycamera_enabled
    subdir_done()
endif

# If ASan is enabled, the link order runtime check will fail as Python is not
# linked to ASan. LD_PRELOAD the ASan runtime if available, or skip the test
# otherwise.

if asan_runtime_missing
    warning('Unable to get path to ASan runtime, Python test disabled')
    subdir_done()
endif

pymod = import('python')
py3 = pymod.find_installation('python3')

pypathdir = meson.project_build_root() / 'src' / 'py'
py_env = ['PYTHONPATH=' + pypathdir]

if asan_enabled
    # Disable leak detection as the Python interpreter is full of leaks.
    py_env += ['LD_PRELOAD=' + asan_runtime, 'ASAN_OPTIONS=detect_leaks=0']
endif

test('pyunittests',
     py3,
     args : files('unittests.py'),
     env : py_env,
     suite : 'pybindings',
     is_parallel : false)
