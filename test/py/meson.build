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

py_env = environment()

pymod = import('python')
py3 = pymod.find_installation('python3')

pypathdir = meson.project_build_root() / 'src' / 'py'
py_env.append('PYTHONPATH', pypathdir)

if asan_enabled
    py_env.append('LD_PRELOAD', asan_runtime)

    # Preload the C++ standard library to work around a bug in ASan when
    # dynamically loading C++ .so modules.
    stdlib = run_command(cxx, '-print-file-name=' + cxx_stdlib + '.so',
                         check : true).stdout().strip()
    py_env.append('LD_PRELOAD', stdlib)

    # Disable leak detection as the Python interpreter is full of leaks.
    py_env.append('ASAN_OPTIONS', 'detect_leaks=0')
endif

test('pyunittests',
     py3,
     args : files('unittests.py'),
     env : py_env,
     suite : 'pybindings',
     is_parallel : false)
