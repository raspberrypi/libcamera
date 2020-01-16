.. section-begin-libcamera

===========
 libcamera
===========

**A complex camera support library for Linux, Android, and ChromeOS**

Cameras are complex devices that need heavy hardware image processing
operations. Control of the processing is based on advanced algorithms that must
run on a programmable processor. This has traditionally been implemented in a
dedicated MCU in the camera, but in embedded devices algorithms have been moved
to the main CPU to save cost. Blurring the boundary between camera devices and
Linux often left the user with no other option than a vendor-specific
closed-source solution.

To address this problem the Linux media community has very recently started
collaboration with the industry to develop a camera stack that will be
open-source-friendly while still protecting vendor core IP. libcamera was born
out of that collaboration and will offer modern camera support to Linux-based
systems, including traditional Linux distributions, ChromeOS and Android.

.. section-end-libcamera
.. section-begin-getting-started

Getting Started
---------------

To fetch the sources, build and install:

::

  git clone git://linuxtv.org/libcamera.git
  cd libcamera
  meson build
  ninja -C build install

Dependencies
~~~~~~~~~~~~

The following Debian/Ubuntu packages are required for building libcamera.
Other distributions may have differing package names:

A C++ toolchain: [required]
	Either {g++, clang}

for libcamera: [required]
        meson (>= 0.47) ninja-build python3-yaml

        If your distribution doesn't provide a recent enough version of meson,
        you can install or upgrade it using pip3.

        .. code::

            pip3 install --user meson
            pip3 install --user --upgrade meson

for device hotplug enumeration: [optional]
	pkg-config libudev-dev

for qcam: [optional]
	qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5

for documentation: [optional]
	python3-sphinx doxygen

.. section-end-getting-started
