.. SPDX-License-Identifier: CC-BY-SA-4.0

==========
 Licenses
==========

TL;DR summary: The libcamera core is covered by the LGPL-2.1-or-later license.
IPA modules included in libcamera are covered by a free software license.
Third-parties may develop IPA modules outside of libcamera and distribute them
under a closed-source license, provided they do not include source code from
the libcamera project.

The libcamera project contains multiple libraries, applications and utilities.
Licenses are expressed through SPDX tags in text-based files that support
comments, and through the .reuse/dep5 file otherwise. A copy of all licenses is
stored in the LICENSES directory.

The following text summarizes the licenses covering the different components of
the project to offer a quick overview for developers. The SPDX and DEP5
information are however authoritative and shall prevail in case of
inconsistencies with the text below.

The libcamera core source code, located under the include/libcamera/ and
src/libcamera/ directories, is fully covered by the LGPL-2.1-or-later license,
which thus covers distribution of the libcamera.so binary. Other files located
in those directories, most notably the meson build files, and various related
build scripts, may be covered by different licenses. None of their source code
is incorporated in the in the libcamera.so binary, they thus don't affect the
distribution terms of the binary.

The IPA modules, located in src/ipa/, are covered by free software licenses
chosen by the module authors. The LGPL-2.1-or-later license is recommended.
Those modules are compiled as separate binaries and dynamically loaded by the
libcamera core at runtime.

The IPA module API is defined in headers located in include/libcamera/ipa/ and
covered by the LGPL-2.1-or-later license. Using the data types (including
classes, structures and enumerations) and macros defined in the IPA module and
libcamera core API headers in IPA modules doesn't extend the LGPL license to
the IPA modules. Third-party closed-source IPA modules are thus permitted,
provided they comply with the licensing requirements of any software they
include or link to.

The libcamera Android camera HAL component is located in src/android/. The
libcamera-specific source code is covered by the LGPL-2.1-or-later license. The
component additionally contains header files and source code, located
respectively in include/android/ and src/android/metadata/, copied verbatim
from Android and covered by the Apache-2.0 license.

The libcamera GStreamer and V4L2 adaptation source code, located respectively
in src/gstreamer/ and src/v4l2/, is fully covered by the LGPL-2.1-or-later
license. Those components are compiled to separate binaries and do not
influence the license of the libcamera core.

The cam and qcam sample applications, as well as the unit tests, located
respectively in src/cam/, src/qcam/ and test/, are covered by the
GPL-2.0-or-later license. qcam additionally includes an icon set covered by the
MIT license. Those applications are compiled to separate binaries and do not
influence the license of the libcamera core.

Additional utilities are located in the utils/ directory and are covered by
various licenses. They are not part of the libcamera core and do not influence
its license.

Finally, copies of various Linux kernel headers are included in include/linux/
to avoid depending on particular versions of those headers being installed in
the system. The Linux kernel headers are covered by their respective license,
including the Linux kernel license syscall exception. Using a copy of those
headers doesn't affect libcamera licensing terms in any way compared to using
the same headers installed in the system from kernel headers packages provided
by Linux distributions.
