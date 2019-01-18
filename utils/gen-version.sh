#!/bin/sh

# SPDX-License-Identifier: GPL-2.0-or-later
# Generate a version string using git describe

if [ -n "$1" ]
then
	cd "$1" 2>/dev/null || exit 1
fi

# Get a short description from the tree.
version=$(git describe --abbrev=8 --match "v[0-9]*" 2>/dev/null)

if [ -z "$version" ]
then
	# Handle an un-tagged repository
	sha=$(git describe --abbrev=8 --always 2>/dev/null)
	commits=$(git log --oneline | wc -l 2>/dev/null)
	version=v0.0.$commits.$sha
fi

# Prevent changed timestamps causing -dirty labels
git update-index --refresh > /dev/null 2>&1
dirty=$(git diff-index --name-only HEAD 2>/dev/null) || dirty=

# Strip the 'g', and replace the preceeding '-' with a '+' to denote a label
version=$(echo "$version" | sed -e 's/-g/+/g')

# Fix the '-' (the patch count) to a '.' as a version increment.
version=$(echo "$version" | sed -e 's/-/./g')

if [ -n "$dirty" ]
then
	version=$version-dirty
fi

echo "$version"
