# SPDX-License-Identifier: CC0-1.0

doc_install_dir = join_paths(get_option('datadir'), 'doc',
                             'libcamera-@0@'.format(libcamera_version))

#
# Doxygen
#

doxygen = find_program('doxygen', required : false)

if doxygen.found()
    cdata = configuration_data()
    cdata.set('VERSION', 'v@0@'.format(libcamera_git_version))
    cdata.set('TOP_SRCDIR', meson.source_root())
    cdata.set('TOP_BUILDDIR', meson.build_root())

    doxyfile = configure_file(input : 'Doxyfile.in',
                              output : 'Doxyfile',
                              configuration : cdata)

    custom_target('doxygen',
                  input : [
                      doxyfile,
                      libcamera_internal_headers,
                      libcamera_ipa_headers,
                      libcamera_public_headers,
                      libcamera_sources,
                      libipa_headers,
                      libipa_sources,
                  ],
                  output : 'api-html',
                  command : [doxygen, doxyfile],
                  install : true,
                  install_dir : doc_install_dir)
endif

#
# Sphinx
#

sphinx = find_program('sphinx-build-3', required : false)
if not sphinx.found()
    sphinx = find_program('sphinx-build', required : false)
endif

if sphinx.found()
    docs_sources = [
       '../README.rst',
        'coding-style.rst',
        'conf.py',
        'contributing.rst',
        'docs.rst',
        'index.rst',
    ]

    release = 'release=v' + libcamera_git_version

    custom_target('documentation',
                  command : [sphinx, '-D', release, '-q', '-W', '-b', 'html',
                             meson.current_source_dir(), '@OUTPUT@'],
                  input : docs_sources,
                  output : 'html',
                  build_by_default : true,
                  install : true,
                  install_dir : doc_install_dir)

    custom_target('documentation-linkcheck',
                  command: [sphinx, '-W', '-b', 'linkcheck', meson.current_source_dir(), '@OUTPUT@'],
                  build_always_stale: true,
                  input: docs_sources,
                  output: 'linkcheck')
endif
