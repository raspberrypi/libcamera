#!/bin/sh

# SPDX-License-Identifier: GPL-2.0-or-later
# Update mojo copy from a chromium source tree

set -e

if [ $# != 1 ] ; then
	echo "Usage: $0 <chromium dir>"
	exit 1
fi

ipc_dir="$(dirname "$(realpath "$0")")/ipc"
chromium_dir="$(realpath "$1")"

cd "${ipc_dir}/../../"

# Reject dirty libcamera trees
if [ -n "$(git status --porcelain -uno)" ] ; then
	echo "libcamera tree is dirty"
	exit 1
fi

if [ ! -d "${chromium_dir}/mojo" ] ; then
	echo "Directory ${chromium_dir} doesn't contain mojo"
	exit 1
fi

if [ ! -d "${chromium_dir}/.git" ] ; then
	echo "Directory ${chromium_dir} doesn't contain a git tree"
	exit 1
fi

# Get the chromium commit id
version=$(git -C "${chromium_dir}" rev-parse --short HEAD)

# Reject dirty chromium trees
if [ -n "$(git -C "${chromium_dir}" status --porcelain)" ] ; then
	echo "Chromium tree in ${chromium_dir} is dirty"
	exit 1
fi

# Remove the previously imported files.
rm -rf utils/ipc/mojo/
rm -rf utils/ipc/tools/

# Copy the diagnosis file
mkdir -p utils/ipc/tools/diagnosis/
cp "${chromium_dir}/tools/diagnosis/crbug_1001171.py" utils/ipc/tools/diagnosis/

# Copy the rest of mojo
mkdir -p utils/ipc/mojo/public/
cp "${chromium_dir}/mojo/public/LICENSE" utils/ipc/mojo/public/

(
	cd "${chromium_dir}" || exit
	find ./mojo/public/tools -type f \
	     -not -path "*/generators/*" \
	     -not -path "*/fuzzers/*" \
	     -exec cp --parents "{}" "${ipc_dir}" ";"
)

# Update the README files
readme=$(cat <<EOF
# SPDX-License-Identifier: CC0-1.0

Files in this directory are imported from ${version} of Chromium. Do not
modify them manually.
EOF
)

echo "$readme" > utils/ipc/mojo/README
echo "$readme" > utils/ipc/tools/README

cat <<EOF
------------------------------------------------------------
mojo updated. Please review and up-port local changes before
committing.
------------------------------------------------------------
EOF
