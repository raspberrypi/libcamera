#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2020, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# ipa-sign.sh - Generate a signature for an IPA module

key="$1"
input="$2"
output="$3"

openssl dgst -sha256 -sign "${key}" -out "${output}" "${input}"
