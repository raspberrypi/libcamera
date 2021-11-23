#!/bin/sh

src_dir="$1"
dst_file="$2"

cat <<EOF > "$dst_file"
/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* This file is auto-generated, do not edit! */
/*
 * Copyright (C) 2018-2019, Google Inc.
 *
 * libcamera.h - libcamera public API
 */

#pragma once

EOF

headers=$(for header in "$src_dir"/*.h "$src_dir"/*.h.in ; do
	header=$(basename "$header")
	header="${header%.in}"
	echo "$header"
done | sort)

for header in $headers ; do
	echo "#include <libcamera/$header>" >> "$dst_file"
done
