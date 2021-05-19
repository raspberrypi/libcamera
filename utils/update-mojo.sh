#!/bin/sh

# SPDX-License-Identifier: GPL-2.0-or-later
# Update mojo copy from a chromium source tree

if [ $# != 1 ] ; then
	echo "Usage: $0 <chromium dir>"
	exit 1
fi

ipc_dir="$(dirname "$(realpath "$0")")/ipc"
chromium_dir="$1"

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

# Reject dirty trees
if [ -n "$(git -C "${chromium_dir}" status --porcelain)" ] ; then
	echo "Chromium tree in ${chromium_dir} is dirty"
	exit 1
fi

# Copy the diagnosis file
cp "${chromium_dir}/tools/diagnosis/crbug_1001171.py" "${ipc_dir}/tools/diagnosis"

# Copy the rest of mojo
cp "${chromium_dir}/mojo/public/LICENSE" "${ipc_dir}/mojo/public"

rm -rf "${ipc_dir}/mojo/public/tools/*"

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

echo "$readme" > "${ipc_dir}/mojo/README"
echo "$readme" > "${ipc_dir}/tools/README"

cat <<EOF
------------------------------------------------------------
mojo updated. Please review and up-port local changes before
committing.
------------------------------------------------------------
EOF
