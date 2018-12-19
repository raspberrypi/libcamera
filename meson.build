project('libcamera', 'c', 'cpp',
  meson_version: '>= 0.40',
  version : '0.1',
  license : 'LGPL 2.1+')

# TODO: Extract this from project.version.
#       Ideally the version at Documentation/conf.py should be
#       generated from this too.
api_version = '0.1'

common_arguments = [
    '-Wall',
    '-Wextra',
    '-Werror',
    '-Wno-unused-parameter',
]

c_arguments = common_arguments
cpp_arguments = common_arguments
cpp_arguments += '-std=c++11'

add_project_arguments(c_arguments, language: 'c')
add_project_arguments(cpp_arguments, language: 'cpp')

libcamera_includes = include_directories('include')

subdir('include')
subdir('src')
subdir('test')
subdir('utils')
subdir('Documentation')

pkg_mod = import('pkgconfig')
pkg_mod.generate(libraries : libcamera,
                 version : '1.0',
                 name : 'libcamera',
                 filebase : 'camera',
                 description : 'Complex Camera Support Library')
