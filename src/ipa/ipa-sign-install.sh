#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2020, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# ipa-sign-install.sh - Regenerate IPA module signatures when installing

libdir=$1
key=$2

ipa_sign=$(dirname "$0")/ipa-sign.sh

echo "Regenerating IPA modules signatures"

for module in "${MESON_INSTALL_DESTDIR_PREFIX}/${libdir}"/*.so ; do
	"${ipa_sign}" "${key}" "${module}" "${module}.sign"
done
