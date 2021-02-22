# SPDX-License-Identifier: CC0-1.0

project('libcamera', 'c', 'cpp',
    meson_version : '>= 0.55',
    version : '0.0.0',
    default_options : [
        'werror=true',
        'warning_level=2',
        'cpp_std=c++17',
    ],
    license : 'LGPL 2.1+')

# Generate version information. The libcamera_git_version variable contains the
# full version with git patch count and SHA1 (e.g. 1.2.3+211-c94a24f4), while
# the libcamera_version variable contains the major.minor.patch (e.g. 1.2.3)
# only. If the source tree isn't under git control, or if it matches the last
# git version tag, the build metadata (e.g. +211-c94a24f4) is omitted from
# libcamera_git_version.
libcamera_git_version = run_command('utils/gen-version.sh',
                                    meson.build_root()).stdout().strip()
if libcamera_git_version == ''
    libcamera_git_version = meson.project_version()
endif

libcamera_version = libcamera_git_version.split('+')[0]

# Configure the build environment.
cc = meson.get_compiler('c')
cxx = meson.get_compiler('cpp')
config_h = configuration_data()

if cc.has_header_symbol('execinfo.h', 'backtrace')
    config_h.set('HAVE_BACKTRACE', 1)
endif

if cc.has_header_symbol('stdlib.h', 'secure_getenv', prefix : '#define _GNU_SOURCE')
    config_h.set('HAVE_SECURE_GETENV', 1)
endif

common_arguments = [
    '-Wshadow',
    '-include', 'config.h',
]

c_arguments = []
cpp_arguments = []

if cc.get_id() == 'clang'
    if cc.version().version_compare('<5')
        error('clang version is too old, libcamera requires 5.0 or newer')
    endif

    # Turn _FORTIFY_SOURCE by default on optimised builds (as it requires -O1
    # or higher). This is needed on clang only as gcc enables it by default.
    if get_option('optimization') != '0'
        common_arguments += [
            '-D_FORTIFY_SOURCE=2',
        ]
    endif

    # Use libc++ by default if available instead of libstdc++ when compiling
    # with clang.
    if cc.find_library('libc++', required: false).found()
        cpp_arguments += [
            '-stdlib=libc++',
        ]
    endif

    cpp_arguments += [
        '-Wextra-semi',
    ]
endif

if cc.get_id() == 'gcc'
    if cc.version().version_compare('<7')
        error('gcc version is too old, libcamera requires 7.0 or newer')
    endif

    # gcc 7.1 introduced processor-specific ABI breakages related to parameter
    # passing on ARM platforms. This generates a large number of messages
    # during compilation with gcc >=7.1. Silence them.
    if (host_machine.cpu_family() == 'arm' and
        cc.version().version_compare('>=7.1'))
        cpp_arguments += [
            '-Wno-psabi',
        ]
    endif
endif

# We use C99 designated initializers for arrays as C++ has no equivalent
# feature. Both gcc and clang support this extension, but recent
# versions of clang generate a warning that needs to be disabled.
if cc.has_argument('-Wno-c99-designator')
    common_arguments += [
        '-Wno-c99-designator',
    ]
endif

c_arguments += common_arguments
cpp_arguments += common_arguments

add_project_arguments(c_arguments, language : 'c')
add_project_arguments(cpp_arguments, language : 'cpp')
add_project_link_arguments(cpp_arguments, language : 'cpp')

libcamera_includes = include_directories('include')

# Sub-directories fill py_modules with their dependencies.
py_modules = []

# Libraries used by multiple components
liblttng = cc.find_library('lttng-ust', required : get_option('tracing'))

# Pipeline handlers
#
# Tests require the vimc pipeline handler, include it automatically when tests
# are enabled.
pipelines = get_option('pipelines')

if get_option('test') and 'vimc' not in pipelines
    message('Enabling vimc pipeline handler to support tests')
    pipelines += ['vimc']
endif

# Utilities are parsed first to provide support for other components.
subdir('utils')

subdir('include')
subdir('src')

# The documentation and test components are optional and can be disabled
# through configuration values. They are enabled by default.

subdir('Documentation')
subdir('test')

if not meson.is_cross_build()
    kernel_version_req = '>= 5.0.0'
    kernel_version = run_command('uname', '-r').stdout().strip()
    if not kernel_version.version_compare(kernel_version_req)
        warning('The current running kernel version @0@ is too old to run libcamera.'
                .format(kernel_version))
        warning('If you intend to use libcamera on this machine, please upgrade to a kernel @0@.'
                .format(kernel_version_req))
    endif
endif

# Create a symlink from the build root to the source root. This is used when
# running libcamera from the build directory to locate resources in the source
# directory (such as IPA configuration files).
run_command('ln', '-fsT', meson.source_root(),
            join_paths(meson.build_root(), 'source'))

configure_file(output : 'config.h', configuration : config_h)

pkg_mod = import('pkgconfig')
pkg_mod.generate(libraries : libcamera,
                 version : '1.0',
                 name : 'libcamera',
                 filebase : 'camera',
                 description : 'Complex Camera Support Library',
                 subdirs : 'libcamera')

# Check for python installation and modules.
py_mod = import('python')
py_mod.find_installation('python3', modules: py_modules)

## Summarise Configurations
summary({
            'Enabled pipelines': pipelines,
        }, section : 'Configuration')
