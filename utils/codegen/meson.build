# SPDX-License-Identifier: CC0-1.0

## Code generation

py_build_env = environment()
# \todo Investigate usage of PYTHONPYCACHEPREFIX for Python >= 3.8
py_build_env.set('PYTHONDONTWRITEBYTECODE', '1')
py_build_env.prepend('PYTHONPATH', meson.current_source_dir())

py_modules += ['jinja2', 'yaml']

gen_controls = files('gen-controls.py')
gen_formats = files('gen-formats.py')
gen_gst_controls = files('gen-gst-controls.py')
gen_header = files('gen-header.sh')
gen_ipa_pub_key = files('gen-ipa-pub-key.py')
gen_tracepoints = files('gen-tp-header.py')

py_mod_controls = files('controls.py')

subdir('ipc')
