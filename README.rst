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

  git clone https://git.libcamera.org/libcamera/libcamera.git
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

for android: [optional]
        libexif-dev libjpeg-dev libyaml-dev

Using GStreamer plugin
~~~~~~~~~~~~~~~~~~~~~~

To use GStreamer plugin from source tree, set the following environment so that
GStreamer can find it. This isn't necessary when libcamera is installed.

  export GST_PLUGIN_PATH=$(pwd)/build/src/gstreamer

The debugging tool ``gst-launch-1.0`` can be used to construct a pipeline and
test it. The following pipeline will stream from the camera named "Camera 1"
onto the OpenGL accelerated display element on your system.

.. code::

  gst-launch-1.0 libcamerasrc camera-name="Camera 1" ! glimagesink

To show the first camera found you can omit the camera-name property, or you
can list the cameras and their capabilities using:

.. code::

  gst-device-monitor-1.0 Video

This will also show the supported stream sizes which can be manually selected
if desired with a pipeline such as:

.. code::

  gst-launch-1.0 libcamerasrc ! 'video/x-raw,width=1280,height=720' ! \
        glimagesink

.. section-end-getting-started

Troubleshooting
~~~~~~~~~~~~~~~

Several users have reported issues with meson installation, crux of the issue
is a potential version mismatch between the version that root uses, and the
version that the normal user uses. On calling `ninja -C build`, it can't find
the build.ninja module. This is a snippet of the error message.

::

  ninja: Entering directory `build'
  ninja: error: loading 'build.ninja': No such file or directory

This can be solved in two ways:

1) Don't install meson again if it is already installed system-wide.

2) If a version of meson which is different from the system-wide version is
already installed, uninstall that meson using pip3, and install again without
the --user argument.
