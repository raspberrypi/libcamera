project('libcamera - supporting complex camera pipelines', 'c', 'cpp',
  version : '0.1',
  license : 'LGPL 2.1+')

common_arguments = [
    '-Wall',
    '-Wextra',
    '-Werror',
    '-Wno-unused-parameter',
]

c_arguments = common_arguments
cpp_arguments = common_arguments

add_project_arguments(c_arguments, language: 'c')
add_project_arguments(cpp_arguments, language: 'cpp')

inc = include_directories('include')

subdir('include')
subdir('lib')
subdir('test')
subdir('utils')

pkg_mod = import('pkgconfig')
pkg_mod.generate(libraries : libcamera,
                 version : '1.0',
                 name : 'libcamera',
                 filebase : 'camera',
                 description : 'Complex Camera Support Library')
