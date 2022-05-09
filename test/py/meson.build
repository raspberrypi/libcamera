# SPDX-License-Identifier: CC0-1.0

if not pycamera_enabled
    subdir_done()
endif

pymod = import('python')
py3 = pymod.find_installation('python3')

pypathdir = meson.project_build_root() / 'src' / 'py'

test('pyunittests',
     py3,
     args : files('unittests.py'),
     env : ['PYTHONPATH=' + pypathdir],
     suite : 'pybindings',
     is_parallel : false)
