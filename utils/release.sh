#!/bin/sh

# SPDX-License-Identifier: GPL-2.0-or-later
# Prepare a project release

set -e

# Abort if we are not within the project root or the tree is not clean.
if [ ! -e utils/gen-version.sh ] || [ ! -e .git ]; then
	echo "This release script must be run from the root of libcamera git tree."
	exit 1
fi

if ! git diff-index --quiet HEAD; then
	echo "Tree must be clean to release."
	exit 1
fi

# Identify current version components
version=$(./utils/gen-version.sh)

# Decide if we are here to bump major, minor, or patch release.
case $1 in
	major|minor|patch)
		bump=$1;
		;;
	*)
		echo "You must specify the version bump level: (major, minor, patch)"
		exit 1
		;;
esac

new_version=$(./utils/semver bump "$bump" "$version")

echo "Bumping $bump"
echo "  Existing version is: $version"
echo "  New version is : $new_version"

# Patch in the version to our meson.build
sed -i -E "s/ version : '.*',/ version : '$new_version',/" meson.build

# Commit the update
git commit meson.build -esm "libcamera v$new_version"

# Create a tag from that commit
git show -s --format=%B | git tag "v$new_version" -s -F -
