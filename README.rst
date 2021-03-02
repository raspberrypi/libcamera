.. SPDX-License-Identifier: CC-BY-SA-4.0

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

Meson Build system: [required]
        meson (>= 0.55) ninja-build pkg-config

        If your distribution doesn't provide a recent enough version of meson,
        you can install or upgrade it using pip3.

        .. code::

            pip3 install --user meson
            pip3 install --user --upgrade meson

for the libcamera core: [required]
        python3-yaml python3-ply python3-jinja2

for IPA module signing: [required]
        libgnutls28-dev openssl

for the Raspberry Pi IPA: [optional]
        libboost-dev

        Support for Raspberry Pi can be disabled through the meson
         'pipelines' option to avoid this dependency.

for device hotplug enumeration: [optional]
	libudev-dev

for documentation: [optional]
	python3-sphinx doxygen graphviz

for gstreamer: [optional]
	libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

for cam: [optional]
        libevent-dev

for qcam: [optional]
	qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 qttools5-dev-tools libtiff-dev

for tracing with lttng: [optional]
        liblttng-ust-dev python3-jinja2 lttng-tools

Using GStreamer plugin
~~~~~~~~~~~~~~~~~~~~~~

To use GStreamer plugin from source tree, set the following environment so that
GStreamer can find it.

  export GST_PLUGIN_PATH=$(pwd)/build/src/gstreamer

The debugging tool `gst-launch-1.0` can be used to construct and pipeline and test
it. The following pipeline will stream from the camera named "Camera 1" onto the
default video display element on your system.

.. code::

  gst-launch-1.0 libcamerasrc camera-name="Camera 1" ! videoconvert ! autovideosink

.. section-end-getting-started
