#!/bin/sh

# SPDX-License-Identifier: GPL-2.0-or-later
#
# On a meson dist run, generate the version string and store it in a file.
# This will later be picked up by the utils/gen-version.sh script and used
# instead of re-generating it. This way, if we are not building in the upstream
# git source tree, the upstream version information will be preserved.

cd "$MESON_SOURCE_ROOT" || return
./utils/gen-version.sh > "$MESON_DIST_ROOT"/.tarball-version
