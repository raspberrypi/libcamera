ipa = shared_module('ipa_vimc', 'vimc.cpp',
                    name_prefix : '',
                    include_directories : [ipa_includes, libipa_includes],
                    dependencies : libcamera_dep,
                    link_with : libipa,
                    install : true,
                    install_dir : ipa_install_dir,
                    cpp_args : '-DLICENSE="LGPL-2.1-or-later"')
