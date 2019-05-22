doc_install_dir = join_paths(get_option('datadir'), 'doc', 'libcamera-@0@'.format(api_version))

#
# Doxygen
#

doxygen = find_program('doxygen', required : false)

if doxygen.found()
    cdata = configuration_data()
    cdata.set('VERSION', meson.project_version())
    cdata.set('TOP_SRCDIR', meson.source_root())
    cdata.set('TOP_BUILDDIR', meson.build_root())

    doxyfile = configure_file(input : 'Doxyfile.in',
                              output : 'Doxyfile',
                              configuration : cdata)

    custom_target('doxygen',
                  input : [
                      doxyfile,
                      libcamera_api,
                      libcamera_headers,
                      libcamera_sources,
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
        'coding-style.rst',
        'conf.py',
        'contributing.rst',
        'docs.rst',
        'index.rst',
    ]

    custom_target('documentation',
                  command : [sphinx, '-q', '-W', '-b', 'html', meson.current_source_dir(), '@OUTPUT@'],
                  input : docs_sources,
                  output : 'html',
                  build_by_default : true,
                  install : true,
                  install_dir : doc_install_dir)
endif
