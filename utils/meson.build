# SPDX-License-Identifier: CC0-1.0

subdir('ipc')
subdir('ipu3')
subdir('tracepoints')

## Code generation
py_modules += ['yaml']
gen_controls = files('gen-controls.py')
gen_formats = files('gen-formats.py')
gen_header = files('gen-header.sh')

## Module signing
gen_ipa_priv_key = files('gen-ipa-priv-key.sh')
gen_ipa_pub_key = files('gen-ipa-pub-key.py')
