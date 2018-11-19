project('libcamera - supporting complex camera pipelines', 'c', 'cpp',
  version : '0.1',
  license : 'LGPL 2.1+')

inc = include_directories('include')

subdir('lib')
subdir('test')
subdir('utils')

pkg_mod = import('pkgconfig')
pkg_mod.generate(libraries : libcamera,
                 version : '1.0',
                 name : 'libcamera',
                 filebase : 'camera',
                 description : 'Complex Camera Support Library')
