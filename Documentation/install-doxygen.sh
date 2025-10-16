#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2025, Ideas on Board Oy
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# Move Doxygen-generated API documentation to correct location

doc_dir="${MESON_INSTALL_DESTDIR_PREFIX}/$1"
shift
dirs="$*"

echo "Moving API documentation"

for dir in $dirs ; do
	rm -r "${doc_dir}/html/${dir}"
	mv "${doc_dir}/${dir}" "${doc_dir}/html/"
done
