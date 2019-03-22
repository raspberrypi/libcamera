project('libcamera', 'c', 'cpp',
  meson_version: '>= 0.40',
  version : '0.1',
  default_options: [
    'werror=true',
    'warning_level=2',
    'cpp_std=c++11',
  ],
  license : 'LGPL 2.1+')

# TODO: Extract this from project.version.
#       Ideally the version at Documentation/conf.py should be
#       generated from this too.
api_version = '0.1'

common_arguments = [
    '-Wno-unused-parameter',
]

c_arguments = common_arguments
cpp_arguments = common_arguments

add_project_arguments(c_arguments, language: 'c')
add_project_arguments(cpp_arguments, language: 'cpp')

libcamera_includes = include_directories('include')

subdir('include')
subdir('src')
subdir('utils')

# The documentation and test components are optional and can be disabled
# through configuration values. They are enabled by default.

if get_option('documentation')
  subdir('Documentation')
endif

if get_option('tests')
  subdir('test')
endif

pkg_mod = import('pkgconfig')
pkg_mod.generate(libraries : libcamera,
                 version : '1.0',
                 name : 'libcamera',
                 filebase : 'camera',
                 description : 'Complex Camera Support Library')
