# SPDX-License-Identifier: CC0-1.0

py_mod = import('python')
py3 = py_mod.find_installation('python3', required : get_option('pycamera'))
pybind11_dep = dependency('pybind11', required : get_option('pycamera'))

pycamera_enabled = py3.found() and pybind11_dep.found()
if not pycamera_enabled
    subdir_done()
endif

subdir('libcamera')

pycamera_devenv = environment()
pycamera_devenv.prepend('PYTHONPATH', meson.current_build_dir())
meson.add_devenv(pycamera_devenv)
