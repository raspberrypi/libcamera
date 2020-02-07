# SPDX-License-Identifier: GPL-2.0-only
# Copyright 2019 Google Inc.

EAPI=6
PYTHON_COMPAT=( python3_{5,6,7} )

inherit git-r3 meson python-any-r1

DESCRIPTION="Camera support library for Linux"
HOMEPAGE="http://libcamera.org"
EGIT_REPO_URI="git://linuxtv.org/libcamera.git"
EGIT_BRANCH="master"

LICENSE="LGPL-2.1+"
SLOT="0"
KEYWORDS="*"
IUSE="udev"

RDEPEND="udev? ( virtual/libudev )"
DEPEND="
	${RDEPEND}
	$(python_gen_any_dep 'dev-python/pyyaml[${PYTHON_USEDEP}]')
"

src_configure() {
	local emesonargs=(
		-Ddocumentation=false
		-Dtests=false
	)
	meson_src_configure
}

src_compile() {
	meson_src_compile
}

src_install() {
	meson_src_install
}
