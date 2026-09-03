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

.. section-begin-cinemate-fork

CineMate fork
-------------

This repository is `CineMate`_'s fork of Raspberry Pi's ``libcamera``, tracked at
https://github.com/Tiramisioux/libcamera.git on the ``cinemate`` branch (this is
also the branch and remote ``cinemate-install.sh`` pins by default via
``LIBCAMERA_REPO_URL`` / ``LIBCAMERA_REPO_REF``). The stock upstream README below
this section is otherwise unmodified; this section is CineMate-specific and is
the first thing a CineMate contributor should read. See the `Getting Started`_
build instructions below for the corrected clone command.

.. _CineMate: https://github.com/Tiramisioux/cinemate

Sensor support
~~~~~~~~~~~~~~

This fork adds two sensors that upstream ``libcamera`` does not carry at
all — ``imx585`` (including the ``imx585_mono`` variant) and ``imx294`` —
and carries materially reworked AE/AWB tuning for a third, ``imx283``,
whose cam helper, base tuning data and sensor-properties entry already ship
upstream (Raspberry Pi added ``imx283`` support in 2024-08). "Support" for
``imx585``/``imx294`` means, per sensor:

- a ``cam_helper_<sensor>.cpp`` pipeline-handler helper (``imx585``:
  ``64570c7``; ``imx294``: ``9790766``) so libcamera's RPi pipeline handler
  can drive the sensor's controls and metadata,
- PiSP and/or VC4 tuning data (AE/AWB/lens-shading tables etc.) under
  ``src/ipa/rpi/{pisp,vc4}/data/``,
- a ``camera_sensor_properties.cpp`` entry (``839b26a``) registering the
  sensor's pixel array size and properties so libcamera recognises it by
  name at all,
- meson build wiring (``da9d3c2``) so the new helpers and tuning data are
  actually compiled and installed.

All three sensors' tuning data was later reworked with materially better
AE/AWB tables (``imx283``: ``80dd06f`` pisp / ``f080f4b`` vc4; ``imx585``:
``9d0cdfe``).

All of the above landed between 2024-08 and 2025-05, well before the
2026-07-05 v3.3.2 release — this is foundational driver support, not a
recent change.

ClearHDR and pixel-rate correctness
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Three fixes here are new since v3.3.2 and are what current CineMate hardware
depends on:

- **16-bit endian-swap fix for ClearHDR** (``bcdd7e17b``, 2026-07-14). The
  RPi PiSP pipeline decided whether to byte-swap 16-bit CFE buffers based
  only on sensor bus bit depth. For ``imx585`` ClearHDR's 16-bit
  ``SRGGB16`` mode with COMP1-compressed CFE output, that meant the swap
  ran anyway, scrambling 8-byte compression blocks two bytes at a time.
  **This corrupted captured ClearHDR data outright**; the fix gates the
  swap on the CFE format not being PiSP1-packed. Tracked in
  ``docs/clear-hdr.md`` and in ``cinepi-raw``'s README compatibility table
  as "16-bit endian swap, gated off compressed formats."
- **PiSP pixel-rate bound derived from the RP1 clock** (``0413c1351`` /
  ``3c7b9abdb``, 2026-08-26). The PiSP IPA's pixel-rate ceiling
  (380 MPix/s stock at 200 MHz RP1 clock, 580 MPix/s with the
  ``rp1-overclock`` overlay's 300 MHz target) is now taken as an explicit
  input via the ``LIBCAMERA_RPI_MAX_PIXEL_RATE`` environment variable,
  falling back to the safe 380 MPix/s stock bound when unset. An earlier
  attempt (``0413c1351``) tried probing the live device-tree clock instead,
  but that failed on real hardware two ways — no ``rp1`` node under
  ``/proc/device-tree`` on CM5 with kernel 6.12.93, and the overlay's
  nominal 300 MHz request is actually delivered as 333.33 MHz — so the
  follow-up commit (``3c7b9abdb``, current branch tip) replaced the probe
  with the explicit env-var input instead. ``cinepi-raw`` sets this variable
  from the same settings switch that enables the overclock overlay, and
  exposes it as its own ``--max-pixel-rate <float>`` flag ("keep advertised
  mode ceilings honest against the live RP1 clock") — the two are meant to
  be read together, not as separate mechanisms.
- **``minPixelProcessingTime`` tuned for RP1 overclock** (``614ce18c6``,
  2026-08-21). First-pass hardcode of ``controller.cpp``'s
  ``minPixelProcessingTime`` to 1.0 µs / 580 MPix/s to match
  ``rp1-overclock``, superseded in behaviour by the pixel-rate-bound work
  above but still the value in effect at that timing constant.

Build fixes (gcc-12 / Raspberry Pi 4)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Two June 2026 commits keep the build green on Bookworm's gcc-12 at
``-O3 -Werror``: ``146e00cb5`` restores upstream's one-shot
``Option::optionName()`` string construction after a two-step version
(``e8b9ff540``) tripped a gcc-12 ``-Wrestrict`` false positive on
Raspberry Pi 4; ``ff24737b6`` additionally demotes ``-Wrestrict`` and
related string-overflow warnings from error to warning in
``src/apps/common/meson.build`` only, so core ``libcamera`` keeps strict
``-Werror`` while the apps layer stays tolerant of future gcc false
positives on other gcc versions.

.. section-end-cinemate-fork

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

.. note::

   **CineMate:** the commands below build stock upstream ``libcamera``. To
   build the branch CineMate actually ships (pinned by
   ``cinemate-install.sh``, required for imx585/imx283/imx294 sensor
   support, the ClearHDR endian-swap fix and the RP1-clock pixel-rate bound
   — see `CineMate fork`_ above), replace the ``git clone`` line below with:

   .. code::

     git clone https://github.com/Tiramisioux/libcamera.git
     cd libcamera
     git checkout cinemate

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
        libtiff-dev qt6-base-dev qt6-tools-dev-tools

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
