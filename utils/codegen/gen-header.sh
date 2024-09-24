#!/bin/sh

dst_file="$1"
shift

cat <<EOF > "$dst_file"
/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* This file is auto-generated, do not edit! */
/*
 * Copyright (C) 2018-2019, Google Inc.
 *
 * libcamera public API
 */

#pragma once

EOF

headers=$(for header in "$@" ; do
	header=$(basename "$header")
	echo "$header"
done | sort)

for header in $headers ; do
	echo "#include <libcamera/$header>" >> "$dst_file"
done
