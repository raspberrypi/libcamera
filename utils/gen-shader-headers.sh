#!/bin/sh

set -e

usage() {
	echo "Usage: $0 <src_dir> <output_header> <shader_file1> [shader_file2 ...]"
	echo
	echo "Generates a C header file containing hex-encoded shader data."
	echo
	echo "Arguments:"
	echo "  src_dir             Path to the base of the source directory"
	echo "  output_header       Path to the generated header file"
	echo "  shader_file(s)      One or more shader files to embed in the header"
	exit 1
}

if [ $# -lt 4 ]; then
	echo "Error: Invalid argument count."
	usage
fi

src_dir="$1"; shift
build_path="$1"; shift

cat <<EOF > "$build_path"
/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* This file is auto-generated, do not edit! */

#pragma once

EOF

cat <<EOF >> "$build_path"
/*
 * List the names of the shaders at the top of
 * header for readability's sake
 *
EOF

for file in "$@"; do
	name=$(basename "$file" | tr '.' '_')
	echo "[SHADER-GEN] $name"
	echo " * unsigned char $name;" >> "$build_path"
done

echo "*/" >> "$build_path"

echo "/* Hex encoded shader data */" >> "$build_path"
for file in "$@"; do
	name=$(basename "$file")
	"$src_dir/utils/gen-shader-header.py" "$name" "$file" >> "$build_path"
	echo >> "$build_path"
done
