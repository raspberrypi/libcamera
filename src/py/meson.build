# SPDX-License-Identifier: CC0-1.0

if meson.is_cross_build()
    py3_dep = dependency('python3', required : get_option('pycamera'))
else
    py3 = import('python').find_installation('python3', required : get_option('pycamera'))
    if not py3.found()
        pycamera_enabled = false
        subdir_done()
    endif

    py3_dep = py3.dependency(required : get_option('pycamera'))
endif

pybind11_dep = dependency('pybind11', required : get_option('pycamera'))

pycamera_enabled = py3_dep.found() and pybind11_dep.found()
if not pycamera_enabled
    subdir_done()
endif

subdir('libcamera')

pycamera_devenv = environment()
pycamera_devenv.prepend('PYTHONPATH', meson.current_build_dir())
meson.add_devenv(pycamera_devenv)
