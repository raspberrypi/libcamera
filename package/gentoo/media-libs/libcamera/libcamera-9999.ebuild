# SPDX-License-Identifier: GPL-2.0-only
# Copyright 2019 Google Inc.

EAPI=6
PYTHON_COMPAT=( python3_{7..10} )

inherit git-r3 meson python-any-r1

DESCRIPTION="Camera support library for Linux"
HOMEPAGE="http://libcamera.org"
EGIT_REPO_URI="https://git.libcamera.org/libcamera/libcamera.git"
EGIT_BRANCH="master"

LICENSE="LGPL-2.1+"
SLOT="0"
KEYWORDS="*"
IUSE="debug doc test udev"

RDEPEND="
	>=net-libs/gnutls-3.3:=
	udev? ( virtual/libudev )
"

DEPEND="
	${RDEPEND}
	dev-libs/openssl
	$(python_gen_any_dep 'dev-python/pyyaml[${PYTHON_USEDEP}]')
"

src_configure() {
	local emesonargs=(
		$(meson_feature doc documentation)
		$(meson_use test)
		--buildtype $(usex debug debug plain)
	)
	meson_src_configure
}

src_compile() {
	meson_src_compile
}

src_install() {
	meson_src_install
}
