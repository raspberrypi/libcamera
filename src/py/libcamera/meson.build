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
    'py_enums.cpp',
    'py_geometry.cpp',
    'py_main.cpp',
])

gen_py_control_enums_input_files = files([
    '../../libcamera/control_ids.yaml',
    'py_control_enums_generated.cpp.in',
])

gen_py_control_enums = files('gen-py-control-enums.py')

pycamera_sources += custom_target('py_gen_controls',
                                  input : gen_py_control_enums_input_files,
                                  output : ['py_control_enums_generated.cpp'],
                                  command : [gen_py_control_enums, '-o', '@OUTPUT@', '@INPUT@'])

gen_py_formats_input_files = files([
    '../../libcamera/formats.yaml',
    'py_formats_generated.cpp.in',
])

gen_py_formats = files('gen-py-formats.py')

pycamera_sources += custom_target('py_gen_formats',
                                  input : gen_py_formats_input_files,
                                  output : ['py_formats_generated.cpp'],
                                  command : [gen_py_formats, '-o', '@OUTPUT@', '@INPUT@'])

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

# Create symlinks from the build dir to the source dir so that we can use the
# Python module directly from the build dir.

run_command('ln', '-fsT', files('__init__.py'),
            meson.current_build_dir() / '__init__.py',
            check: true)

install_data(['__init__.py'], install_dir : destdir)

# \todo Generate stubs when building. See https://peps.python.org/pep-0484/#stub-files
# Note: Depends on pybind11-stubgen. To generate pylibcamera stubs:
# $ PYTHONPATH=build/src/py pybind11-stubgen --no-setup-py -o build/src/py libcamera
