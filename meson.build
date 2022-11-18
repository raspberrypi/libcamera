# SPDX-License-Identifier: CC0-1.0

project('libcamera', 'c', 'cpp',
    meson_version : '>= 0.56',
    version : '0.0.2',
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
                                    meson.project_build_root(),
                                    meson.project_source_root(),
                                    check: false).stdout().strip()
if libcamera_git_version == ''
    libcamera_git_version = meson.project_version()
endif

libcamera_version = libcamera_git_version.split('+')[0]

# A shallow clone, or a clone without a reachable tag equivalent to the
# meson.project_version() could leave the project in a mis-described state.
# Produce a warning in this event, and fix to a best effort.
if libcamera_version != meson.project_version()
    warning('The sources disagree about the version: '
            + libcamera_version + ' != ' + meson.project_version())

    summary({'libcamera git version' : libcamera_git_version,
             'Source version match' : false,
            },
            bool_yn : true, section : 'Versions')

    # Replace the version components reported by git with the release version,
    # but keep all trailing information supplied by git.
    libcamera_git_version = (meson.project_version() +
                             libcamera_git_version.strip(libcamera_version))
    libcamera_version = meson.project_version()

    # Append a marker to show we have modified this version string
    libcamera_git_version += '-nvm'
endif

# Until we make ABI compatible releases, the full libcamera version is used as
# the soname. No ABI/API compatibility is guaranteed between releases (x.y.z).
#
# When automatic ABI based detection is used to increment the version, this
# will bump the minor number (x.y).
#
# When we declare a stable ABI/API we will provide a 1.0 release and the
# soversion at that point will be the 'major' release value (x).
libcamera_soversion = libcamera_version

summary({ 'Sources': libcamera_git_version, }, section : 'Versions')

# This script generates the .tarball-version file on a 'meson dist' command.
meson.add_dist_script('utils/run-dist.sh')

# Configure the build environment.
cc = meson.get_compiler('c')
cxx = meson.get_compiler('cpp')
config_h = configuration_data()

if cc.has_header_symbol('unistd.h', 'issetugid')
    config_h.set('HAVE_ISSETUGID', 1)
endif

if cc.has_header_symbol('stdlib.h', 'secure_getenv', prefix : '#define _GNU_SOURCE')
    config_h.set('HAVE_SECURE_GETENV', 1)
endif

common_arguments = [
    '-Wshadow',
    '-include', meson.current_build_dir() / 'config.h',
]

c_arguments = []
cpp_arguments = []

if cc.get_id() == 'clang'
    if cc.version().version_compare('<9')
        error('clang version is too old, libcamera requires 9.0 or newer')
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
        '-Wthread-safety',
    ]
endif

if cc.get_id() == 'gcc'
    if cc.version().version_compare('<8')
        error('gcc version is too old, libcamera requires 8.0 or newer')
    endif

    # On gcc 8, the file system library is provided in a separate static
    # library.
    if cc.version().version_compare('<9')
        cpp_arguments += [
            '-lstdc++fs',
        ]
    endif

    # gcc 7.1 introduced processor-specific ABI breakages related to parameter
    # passing on ARM platforms. This generates a large number of messages
    # during compilation. Silence them.
    if host_machine.cpu_family() == 'arm'
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
liblttng = dependency('lttng-ust', required : get_option('tracing'))

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
    kernel_version = run_command('uname', '-r', check: true).stdout().strip()
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
run_command('ln', '-fsT', meson.project_source_root(), meson.project_build_root() / 'source',
            check: true)

configure_file(output : 'config.h', configuration : config_h)

# Check for python installation and modules.
py_mod = import('python')
py_mod.find_installation('python3', modules: py_modules)

## Summarise Configurations
summary({
            'Enabled pipelines': pipelines,
            'Enabled IPA modules': enabled_ipa_modules,
            'Tracing support': tracing_enabled,
            'Android support': android_enabled,
            'GStreamer support': gst_enabled,
            'Python bindings': pycamera_enabled,
            'V4L2 emulation support': v4l2_enabled,
            'cam application': cam_enabled,
            'qcam application': qcam_enabled,
            'lc-compliance application': lc_compliance_enabled,
            'Unit tests': test_enabled,
        },
        section : 'Configuration',
        bool_yn : true)
