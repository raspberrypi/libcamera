#!/bin/sh

# SPDX-License-Identifier: GPL-2.0-or-later
# Generate a version string using git describe

build_dir="$1"

# Bail out if the directory isn't under git control
src_dir=$(git rev-parse --git-dir 2>&1) || exit 1
src_dir=$(readlink -f "$src_dir/..")

# Get a short description from the tree.
version=$(git describe --abbrev=8 --match "v[0-9]*" 2>/dev/null)

if [ -z "$version" ]
then
	# Handle an un-tagged repository
	sha=$(git describe --abbrev=8 --always 2>/dev/null)
	commits=$(git log --oneline | wc -l 2>/dev/null)
	version="v0.0.0-$commits-g$sha"
fi

# Append a '-dirty' suffix if the working tree is dirty. Prevent false
# positives due to changed timestamps by running git update-index.
if [ -z "$build_dir" ] || (echo "$build_dir" | grep -q "$src_dir")
then
	git update-index --refresh > /dev/null 2>&1
fi
git diff-index --quiet HEAD || version="$version-dirty ($(date --iso-8601=seconds))"

# Replace first '-' with a '+' to denote build metadata, strip the 'g' in from
# of the git SHA1 and remove the initial 'v'.
version=$(echo "$version" | sed -e 's/-/+/' | sed -e 's/-g/-/' | cut -c 2-)

echo "$version"
