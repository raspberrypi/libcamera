#!/bin/bash

# SPDX-License-Identifier: GPL-2.0-or-later
# Generate and compare the ABI compatibilty of two libcamera versions

name=$(basename "$0")

usage() {
	cat << EOF
$name: Determine the ABI/API compatibility of two build versions

  $name [--help] [--abi-dir=<PATH>] [--tmp-dir=<PATH>] ARGS

The positional arguments (ARGS) determine the versions that will be compared and
take three variants:

  - No positional arguments:
      $name [optional arguments]

      It is assumed to compare the current git HEAD against the most recent TAG

  - One positional argument:
      $name [optional aguments] COMMITISH

      The given COMMITISH is compared against it's most recent TAG

  - Two positional arguments:
      $name [optional aguments] BASE COMMITISH

      The given COMMITISH is compared against the given BASE.

Optional Arguments:
  --abi-dir <path> Use <path> for storing (or retrieving existing) ABI data
                   files

  --tmp-dir <path> Specify temporary build location for building ABI data.
                   This could be a tmpfs/RAM disk to save on disk writes.
EOF
}

dbg () {
	echo "$@" >&2
}

die () {
	echo "$name: $*" >&2
	exit 1
}

describe () {
	git describe --tags "$1" \
		|| die "Failed to describe $1"
}

prev_release () {
	git describe --tags --abbrev=0 "$1"^ \
		|| die "Failed to identify previous release tag from $1"
}

# Make sure we exit on errors during argument parsing.
set -Eeuo pipefail

positional=()
while [[ $# -gt 0 ]] ; do
	option="$1"
	shift

	case $option in
	-h|--help)
		usage
		exit 0
	        ;;

	--abi-dir)
		abi_dir=$1
		shift
		;;

	--tmp-dir)
		tmp=$1
		shift
		;;

	-*)
		die "Unrecognised argument $option"
		;;

	*) # Parse unidentified arguments based on position.
		positional+=("$option")
		;;
	esac
done
set -- "${positional[@]}" # restore positional parameters.

# Parse positional arguments.
case $# in
	0) 	# Check HEAD against previous 'release'.
		from=$(prev_release HEAD)
		to=$(describe HEAD)
		;;

	1)	# Check COMMIT against previous release.
		from=$(prev_release "$1")
		to=$(describe "$1")
		;;

	2)	# Check ABI between FROM and TO explicitly.
		from=$(describe "$1")
		to=$(describe "$2")
		;;

	*)
		die "Invalid arguments"
		;;
esac

if ! which abi-compliance-checker; then
	die "This tool requires 'abi-compliance-checker' to be installed."
fi


abi_dir=${abi_dir:-abi}
tmp=${tmp:-"$abi_dir/tmp/"}

echo "Validating ABI compatibility between $from and $to"

mkdir -p "$abi_dir"
mkdir -p "$tmp"

# Generate an abi-compliance-checker xml description file.
create_xml() {
	local output="$1"
	local version="$2"
	local root="$3"

	echo "<version>$version</version>" > "$output"
	echo "<headers>$root/usr/local/include/</headers>" >> "$output"
	echo "<libs>$root/usr/local/lib/</libs>" >> "$output"
}

# Check if an ABI dump file exists, and if not create one by building a minimal
# configuration of libcamera at the specified version using a clean worktree.
create_abi_dump() {
	local version="$1"
	local abi_file="$abi_dir/$version.abi.dump"
	local worktree="$tmp/$version"
	local build="$tmp/$version-build"

	# Use a fully qualified path when calling ninja -C.
	install=$(realpath "$tmp/$version-install")

	if [[ ! -e "$abi_file" ]] ; then
		dbg "Creating ABI dump for $version in $abi_dir"
		git worktree add --force "$worktree" "$version"

		# Generate a minimal libcamera build. "lib" and "prefix" are
		# defined explicitly to avoid system default ambiguities.
		meson setup "$build" "$worktree" \
			-Dcam=disabled \
			-Ddocumentation=disabled \
			-Dgstreamer=disabled \
			-Dlc-compliance=disabled \
			-Dlibdir=lib \
			-Dpipelines= \
			-Dprefix=/usr/local/ \
			-Dpycamera=disabled \
			-Dqcam=disabled \
			-Dtracing=disabled

		ninja -C "$build"
		DESTDIR="$install" ninja -C "$build" install

		# Create an xml descriptor with parameters to generate the dump file.
		create_xml \
			"$install/libcamera-abi-dump.xml" \
			"$version" \
			"$install"

		abi-compliance-checker \
			-lib libcamera \
			-v1 "$version" \
			-dump "$install/libcamera-abi-dump.xml" \
			-dump-path "$abi_file"

		dbg Created "$abi_file"

		dbg Removing Worktree "$worktree"
		git worktree remove -f "$worktree"

		dbg Removing "$build"
		rm -r "$build"

		dbg Removing "$install"
		rm -r "$install"
	fi
}

# Create the requested ABI dump files if they don't yet exist.
create_abi_dump "$from"
create_abi_dump "$to"

# TODO: Future iterations and extensions here could add "-stdout -xml" and
# parse the results automatically.
abi-compliance-checker -l libcamera \
	-old "$abi_dir/$from.abi.dump" \
	-new "$abi_dir/$to.abi.dump"

# On (far too many) occasions, the tools keep running leaving a cpu core @ 100%
# CPU usage. Perhaps some subprocess gets launched but never rejoined. Stop
# them all.
#
# TODO: Investigate this and report upstream.
killall abi-compliance-checker 2>/dev/null
