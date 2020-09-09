#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2020, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# gen-ipa-priv-key.sh - Generate an RSA private key to sign IPA modules

key="$1"

openssl genpkey -algorithm RSA -out "${key}" -pkeyopt rsa_keygen_bits:2048
