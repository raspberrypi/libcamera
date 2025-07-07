.. SPDX-License-Identifier: CC-BY-SA-4.0

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

.. section-begin-getting-started

Getting Started
---------------

Only build ``libcamera`` from scratch if you need custom behaviour or the latest features that have not yet reached ``apt`` repositories.

If you run ``Raspberry Pi OS Lite``, begin by installing the following packages:
  
.. code::

  sudo apt install -y python-pip git python3-jinja2

First, install the following ``libcamera`` dependencies:
.. code::

  sudo apt install -y libboost-dev
  sudo apt install -y libgnutls28-dev openssl libtiff-dev pybind11-dev
  sudo apt install -y qtbase5-dev libqt5core5a libqt5widgets
  sudo apt install -y meson cmake
  sudo apt install -y python3-yaml python3-ply
  sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev

Now we're ready to build ``libcamera`` itself.

Download a local copy of Raspberry Pi's fork of ``libcamera`` from GitHub, before building and installing freshly-build binary:

.. code::

  git clone https://github.com/raspberrypi/libcamera.git
  cd libcamera
  meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
  ninja -C build install

You can disable the ``gstreamer`` plugin by replacing ``-Dgstreamer=enabled`` with ``-Dgstreamer=disabled`` during the ``meson`` build configuration.
If you disable ``gstreamer``, there is no need to install the ``libglib2.0-dev`` and ``libgstreamer-plugins-base1.0-dev`` dependencies.

On devices with 1GB of memory or less, the build may exceed available memory. Append the ``-j 1`` flag to ``ninja`` commands to limit the build to a single process.
This should prevent the build from exceeding available memory on devices like the Raspberry Pi Zero and the Raspberry Pi 3. 

``libcamera`` does not yet have a stable binary interface. Always build ``rpicam-apps`` after you build ``libcamera``.

You can find more informations at `Raspberry Pi libcamera documentation`_ pages.

.. _Raspberry Pi libcamera documentation: https://www.raspberrypi.com/documentation/computers/camera_software.html

Dependencies
~~~~~~~~~~~~

The following Debian/Ubuntu packages are required for building libcamera.
Other distributions may have differing package names:

A C++ toolchain: [required]
        Either {g++, clang}

Meson Build system: [required]
        meson (>= 0.63) ninja-build pkg-config

for the libcamera core: [required]
        libyaml-dev python3-yaml python3-ply python3-jinja2

for IPA module signing: [recommended]
        Either libgnutls28-dev or libssl-dev, openssl

        Without IPA module signing, all IPA modules will be isolated in a
        separate process. This adds an unnecessary extra overhead at runtime.

for improved debugging: [optional]
        libdw-dev libunwind-dev

        libdw and libunwind provide backtraces to help debugging assertion
        failures. Their functions overlap, libdw provides the most detailed
        information, and libunwind is not needed if both libdw and the glibc
        backtrace() function are available.

for device hotplug enumeration: [optional]
        libudev-dev

for documentation: [optional]
        python3-sphinx doxygen graphviz texlive-latex-extra

for gstreamer: [optional]
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

for Python bindings: [optional]
        libpython3-dev pybind11-dev

for cam: [optional]
        libevent-dev is required to support cam, however the following
        optional dependencies bring more functionality to the cam test
        tool:

        - libdrm-dev: Enables the KMS sink
        - libjpeg-dev: Enables MJPEG on the SDL sink
        - libsdl2-dev: Enables the SDL sink
        - libtiff-dev: Enables writing DNG

for qcam: [optional]
        libtiff-dev qt6-base-dev

for tracing with lttng: [optional]
        liblttng-ust-dev python3-jinja2 lttng-tools

for android: [optional]
        libexif-dev libjpeg-dev

for lc-compliance: [optional]
        libevent-dev libgtest-dev

for abi-compat.sh: [optional]
        abi-compliance-checker

Basic testing with cam utility
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``cam`` utility can be used for basic testing. You can list the cameras
detected on the system with ``cam -l``, and capture ten frames from the first
camera and save them to disk with ``cam -c 1 --capture=10 --file``. See
``cam -h`` for more information about the ``cam`` tool.

In case of problems, a detailed debug log can be obtained from libcamera by
setting the ``LIBCAMERA_LOG_LEVELS`` environment variable:

.. code::

    :~$ LIBCAMERA_LOG_LEVELS=*:DEBUG cam -l

Using GStreamer plugin
~~~~~~~~~~~~~~~~~~~~~~

To use the GStreamer plugin from the source tree, use the meson ``devenv``
command.  This will create a new shell instance with the ``GST_PLUGIN_PATH``
environment set accordingly.

.. code::

  meson devenv -C build

The debugging tool ``gst-launch-1.0`` can be used to construct a pipeline and
test it. The following pipeline will stream from the camera named "Camera 1"
onto the OpenGL accelerated display element on your system.

.. code::

  gst-launch-1.0 libcamerasrc camera-name="Camera 1" ! queue ! glimagesink

To show the first camera found you can omit the camera-name property, or you
can list the cameras and their capabilities using:

.. code::

  gst-device-monitor-1.0 Video

This will also show the supported stream sizes which can be manually selected
if desired with a pipeline such as:

.. code::

  gst-launch-1.0 libcamerasrc ! 'video/x-raw,width=1280,height=720' ! \
       queue ! glimagesink

The libcamerasrc element has two log categories, named libcamera-provider (for
the video device provider) and libcamerasrc (for the operation of the camera).
All corresponding debug messages can be enabled by setting the ``GST_DEBUG``
environment variable to ``libcamera*:7``.

Presently, to prevent element negotiation failures it is required to specify
the colorimetry and framerate as part of your pipeline construction. For
instance, to capture and encode as a JPEG stream and receive on another device
the following example could be used as a starting point:

.. code::

   gst-launch-1.0 libcamerasrc ! \
        video/x-raw,colorimetry=bt709,format=NV12,width=1280,height=720,framerate=30/1 ! \
        queue ! jpegenc ! multipartmux ! \
        tcpserversink host=0.0.0.0 port=5000

Which can be received on another device over the network with:

.. code::

   gst-launch-1.0 tcpclientsrc host=$DEVICE_IP port=5000 ! \
        multipartdemux ! jpegdec ! autovideosink

The GStreamer element also supports multiple streams. This is achieved by
requesting additional source pads. Downstream caps filters can be used
to choose specific parameters like resolution and pixel format. The pad
property ``stream-role`` can be used to select a role.

The following example displays a 640x480 view finder while streaming JPEG
encoded 800x600 video. You can use the receiver pipeline above to view the
remote stream from another device.

.. code::

   gst-launch-1.0 libcamerasrc name=cs src::stream-role=view-finder src_0::stream-role=video-recording \
       cs.src ! queue ! video/x-raw,width=640,height=480 ! videoconvert ! autovideosink \
       cs.src_0 ! queue ! video/x-raw,width=800,height=600 ! videoconvert ! \
       jpegenc ! multipartmux ! tcpserversink host=0.0.0.0 port=5000

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

1. Don't install meson again if it is already installed system-wide.

2. If a version of meson which is different from the system-wide version is
   already installed, uninstall that meson using pip3, and install again without
   the --user argument.
