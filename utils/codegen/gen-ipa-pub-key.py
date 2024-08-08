#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2020, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# Generate the IPA module signing public key

import string
import subprocess
import sys


def main(argv):
    if len(argv) != 4:
        print('Usage: %s priv-key template output' % argv[0])
        return 1

    priv_key = argv[1]
    template = argv[2]
    output = argv[3]

    try:
        ret = subprocess.run(['openssl', 'rsa', '-pubout', '-in', priv_key,
                              '-outform', 'DER'],
                             stdout=subprocess.PIPE)
    except FileNotFoundError:
        print('Please install openssl to sign IPA modules')
        return 1

    ipa_key = ['0x%02x' % c for c in ret.stdout]
    ipa_key = [', '.join(ipa_key[bound:bound + 8]) for bound in range(0, len(ipa_key), 8)]
    ipa_key = ',\n\t'.join(ipa_key)
    data = {'ipa_key': ipa_key}

    template = open(template, 'rb').read()
    template = template.decode('utf-8')
    template = string.Template(template)

    f = open(output, 'wb')
    f.write(template.substitute(data).encode('utf-8'))
    f.close()

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
