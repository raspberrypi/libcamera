sphinx = find_program('sphinx-build-3', required: false)
if not sphinx.found()
    sphinx = find_program('sphinx-build', required: false)
endif

if sphinx.found()
    docs_sources = [
        'conf.py',
        'contributing.rst',
        'index.rst',
    ]

    custom_target('documentation',
                  command: [sphinx, '-W', '-b', 'html', meson.current_source_dir(), '@OUTPUT@'],
                  input: docs_sources,
                  output: 'html',
                  build_by_default: true,
                  install: true,
                  install_dir: 'share/doc/libcamera-@0@'.format(api_version))
endif
