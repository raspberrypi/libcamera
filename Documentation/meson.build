# SPDX-License-Identifier: CC0-1.0

doc_install_dir = get_option('datadir') / 'doc' / 'libcamera-@0@'.format(libcamera_version)

#
# Doxygen
#

doxygen = find_program('doxygen', required : get_option('documentation'))
dot = find_program('dot', required : get_option('documentation'))

if doxygen.found() and dot.found()
    cdata = configuration_data()
    cdata.set('VERSION', 'v@0@'.format(libcamera_git_version))
    cdata.set('TOP_SRCDIR', meson.project_source_root())
    cdata.set('TOP_BUILDDIR', meson.project_build_root())
    cdata.set('OUTPUT_DIR', meson.current_build_dir())
    cdata.set('WARN_AS_ERROR', get_option('doc_werror') ? 'YES' : 'NO')

    doxygen_predefined = []
    foreach key : config_h.keys()
        doxygen_predefined += '@0@=@1@'.format(key, config_h.get(key))
    endforeach

    cdata.set('PREDEFINED', ' \\\n\t\t\t '.join(doxygen_predefined))

    doxyfile_common = configure_file(input : 'Doxyfile-common.in',
                                     output : 'Doxyfile-common',
                                     configuration : cdata)

    doxygen_public_input = [
        libcamera_base_public_headers,
        libcamera_base_public_sources,
        libcamera_public_headers,
        libcamera_public_sources,
    ]

    doxygen_internal_input = [
        libcamera_base_private_headers,
        libcamera_base_internal_sources,
        libcamera_internal_headers,
        libcamera_internal_sources,
        libcamera_ipa_headers,
        libcamera_ipa_interfaces,
        libipa_headers,
        libipa_sources,
    ]

    if is_variable('ipu3_ipa_sources')
        doxygen_internal_input += [ipu3_ipa_sources]
    endif

    # We run doxygen twice - the first run excludes internal API objects as it
    # is intended to document the public API only. A second run covers all of
    # the library's objects for libcamera developers. Common configuration is
    # set in an initially generated Doxyfile, which is then included by the two
    # final Doxyfiles.

    # This is the "public" run of doxygen generating an abridged version of the
    # API's documentation.

    doxyfile_tmpl = configure_file(input : 'Doxyfile-public.in',
                                   output : 'Doxyfile-public.tmpl',
                                   configuration : cdata)

    # The set of public input files stored in the doxygen_public_input array
    # needs to be set in Doxyfile public. We can't pass them through cdata
    # cdata, as some of the array members are custom_tgt instances, which
    # configuration_data.set() doesn't support. Using a separate script invoked
    # through custom_target(), which supports custom_tgt instances as inputs.

    doxyfile = custom_target('doxyfile-public',
                             input : [
                                 doxygen_public_input,
                             ],
                             output : 'Doxyfile-public',
                             command : [
                                 'gen-doxyfile.py',
                                 '-o', '@OUTPUT@',
                                 doxyfile_tmpl,
                                 '@INPUT@',
                             ])

    custom_target('doxygen-public',
                  input : [
                      doxyfile,
                      doxyfile_common,
                  ],
                  output : 'api-html',
                  command : [doxygen, doxyfile],
                  install : true,
                  install_dir : doc_install_dir,
                  install_tag : 'doc')

    # This is the internal documentation, which hard-codes a list of directories
    # to parse in its doxyfile.

    doxyfile = configure_file(input : 'Doxyfile-internal.in',
                              output : 'Doxyfile-internal',
                              configuration : cdata)

    custom_target('doxygen-internal',
                  input : [
                      doxyfile,
                      doxyfile_common,
                      doxygen_internal_input,
                  ],
                  output : 'internal-api-html',
                  command : [doxygen, doxyfile],
                  install : true,
                  install_dir : doc_install_dir,
                  install_tag : 'doc-internal')
endif

#
# Sphinx
#

sphinx = find_program('sphinx-build-3', 'sphinx-build',
                      required : get_option('documentation'))

if sphinx.found()
    docs_sources = [
        'camera-sensor-model.rst',
        'code-of-conduct.rst',
        'coding-style.rst',
        'conf.py',
        'contributing.rst',
        'design/ae.rst',
        'documentation-contents.rst',
        'environment_variables.rst',
        'feature_requirements.rst',
        'guides/application-developer.rst',
        'guides/ipa.rst',
        'guides/pipeline-handler.rst',
        'guides/tracing.rst',
        'index.rst',
        'introduction.rst',
        'lens_driver_requirements.rst',
        'libcamera_architecture.rst',
        'mali-c55.dot',
        'python-bindings.rst',
        'sensor_driver_requirements.rst',
        'software-isp-benchmarking.rst',
       '../README.rst',
    ]

    release = 'release=v' + libcamera_git_version

    custom_target('documentation',
                  command : [sphinx, '-D', release, '-q', '-W', '-b', 'html',
                             meson.current_source_dir(), '@OUTPUT@'],
                  input : docs_sources,
                  output : 'html',
                  build_by_default : true,
                  install : true,
                  install_dir : doc_install_dir,
                  install_tag : 'doc')

    custom_target('documentation-linkcheck',
                  command : [sphinx, '-W', '-b', 'linkcheck', meson.current_source_dir(), '@OUTPUT@'],
                  build_always_stale : true,
                  input : docs_sources,
                  output : 'linkcheck')
endif
