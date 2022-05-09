# SPDX-License-Identifier: CC0-1.0

py3_dep = dependency('python3', required : get_option('pycamera'))

if not py3_dep.found()
    pycamera_enabled = false
    subdir_done()
endif

pycamera_enabled = true

pybind11_proj = subproject('pybind11')
pybind11_dep = pybind11_proj.get_variable('pybind11_dep')

pycamera_sources = files([
    'pyenums.cpp',
    'pymain.cpp',
])

gen_input_files = [
    meson.project_source_root() / 'src' / 'libcamera' / 'control_ids.yaml',
    'pyenums_generated.cpp.in',
]

generated_sources = custom_target('py_gen_controls',
                                  input : gen_input_files,
                                  output : ['pyenums_generated.cpp'],
                                  command : ['gen-py-control-enums.py', '-o', '@OUTPUT@', '@INPUT@'])

pycamera_sources += generated_sources

pycamera_deps = [
    libcamera_public,
    py3_dep,
    pybind11_dep,
]

pycamera_args = [
    '-fvisibility=hidden',
    '-Wno-shadow',
    '-DPYBIND11_USE_SMART_HOLDER_AS_DEFAULT',
    '-DLIBCAMERA_BASE_PRIVATE',
]

destdir = get_option('libdir') / ('python' + py3_dep.version()) / 'site-packages' / 'libcamera'

pycamera = shared_module('_libcamera',
                         pycamera_sources,
                         install : true,
                         install_dir : destdir,
                         name_prefix : '',
                         dependencies : pycamera_deps,
                         cpp_args : pycamera_args)

run_command('ln', '-fsT', '../../../../src/py/libcamera/__init__.py',
            meson.current_build_dir() / '__init__.py',
            check: true)

install_data(['__init__.py'], install_dir : destdir)

# \todo Generate stubs when building. See https://peps.python.org/pep-0484/#stub-files
# Note: Depends on pybind11-stubgen. To generate pylibcamera stubs:
# $ PYTHONPATH=build/src/py pybind11-stubgen --no-setup-py -o build/src/py libcamera
# $ mv build/src/py/libcamera-stubs/* build/src/py/libcamera/
