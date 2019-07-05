#!/bin/sh

# SPDX-License-Identifier: GPL-2.0-or-later
# Generate a version string using git describe

if [ -n "$1" ]
then
	cd "$1" 2>/dev/null || exit 1
fi

# Bail out if the directory isn't under git control
git rev-parse --git-dir >/dev/null 2>&1 || exit 1

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
git update-index --refresh > /dev/null 2>&1
git diff-index --quiet HEAD || version="$version-dirty"

# Replace first '-' with a '+' to denote build metadata, strip the 'g' in from
# of the git SHA1 and remove the initial 'v'.
version=$(echo "$version" | sed -e 's/-/+/' | sed -e 's/-g/-/' | cut -c 2-)

echo "$version"
