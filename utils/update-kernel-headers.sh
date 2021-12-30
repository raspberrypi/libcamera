#!/bin/sh

# SPDX-License-Identifier: GPL-2.0-or-later
# Update the kernel headers copy from a kernel source tree

if [ $# != 1 ] ; then
	echo "Usage: $0 <kernel dir>"
	exit 1
fi

header_dir="$(dirname "$(realpath "$0")")/../include/linux"
kernel_dir="$1"

# Bail out if the directory doesn't contain kernel sources
line=$(head -3 "${kernel_dir}/Kbuild" 2>/dev/null | tail -1)
if [ "$line" != "# Kbuild for top-level directory of the kernel" ] ; then
	echo "Directory ${kernel_dir} doesn't contain a kernel source tree"
	exit 1
fi

if [ ! -d "${kernel_dir}/.git" ] ; then
	echo "Directory ${kernel_dir} doesn't contain a git tree"
	exit 1
fi

# Check the kernel version, and reject dirty trees
version=$(git -C "${kernel_dir}" describe --dirty)
echo $version
if echo "${version}" | grep -q dirty ; then
	echo "Kernel tree in ${kernel_dir} is dirty"
	exit 1
fi

# Install the headers to a temporary directory
install_dir=$(mktemp -d)
if [ ! -d "${install_dir}" ] ; then
	echo "Failed to create temporary directory"
	exit 1
fi

trap "rm -rf ${install_dir}" EXIT

set -e
make -C "${kernel_dir}" O="${install_dir}" headers_install
set +e

# Copy the headers
headers="
	drm/drm_fourcc.h
	linux/dma-buf.h
	linux/dma-heap.h
	linux/media-bus-format.h
	linux/media.h
	linux/rkisp1-config.h
	linux/v4l2-common.h
	linux/v4l2-controls.h
	linux/v4l2-mediabus.h
	linux/v4l2-subdev.h
	linux/videodev2.h
"

for header in $headers ; do
	name=$(basename "${header}")
	cp "${install_dir}/usr/include/${header}" "${header_dir}/${name}"
done

# The IPU3 header is a special case, as it's stored in staging. Handle it
# manually.
(cd "${install_dir}" ; "${kernel_dir}/scripts/headers_install.sh" \
	"${kernel_dir}/drivers/staging/media/ipu3/include/uapi/intel-ipu3.h" \
	"${header_dir}/intel-ipu3.h")

# Update the README file
cat <<EOF > "${header_dir}/README"
# SPDX-License-Identifier: CC0-1.0

Files in this directory are imported from ${version} of the Linux kernel. Do not
modify them manually.
EOF

# Cleanup
rm -rf "${install_dir}"

cat <<EOF
----------------------------------------------------------------------
Kernel headers updated. Please review and up-port local changes before
committing.
----------------------------------------------------------------------
EOF
