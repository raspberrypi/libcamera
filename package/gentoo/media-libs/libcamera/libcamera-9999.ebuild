# Copyright 2019 Google Inc.
# Distributed under the terms of the GNU General Public License v2

EAPI=6
inherit git-r3 meson

DESCRIPTION="Camera support library for Linux"
HOMEPAGE="http://libcamera.org"
EGIT_REPO_URI="git://linuxtv.org/libcamera.git"
EGIT_BRANCH="master"

LICENSE="LGPL-2.1+"
SLOT="0"
KEYWORDS="*"
IUSE="udev"

RDEPEND="udev? ( virtual/libudev )"
DEPEND="${RDEPEND}"

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
