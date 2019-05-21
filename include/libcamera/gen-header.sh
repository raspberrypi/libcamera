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
#ifndef __LIBCAMERA_LIBCAMERA_H__
#define __LIBCAMERA_LIBCAMERA_H__

EOF

for header in "$src_dir"/*.h ; do
	header=$(basename "$header")
	echo "#include <libcamera/$header>" >> "$dst_file"
done

cat <<EOF >> "$dst_file"

#endif /* __LIBCAMERA_LIBCAMERA_H__ */
EOF
