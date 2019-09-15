rkisp1_ipa = shared_module('ipa_rkisp1',
                           'rkisp1.cpp',
                           name_prefix : '',
                           include_directories : [ipa_includes, libipa_includes],
                           dependencies : libcamera_dep,
                           link_with : libipa,
                           install : true,
                           install_dir : ipa_install_dir)
