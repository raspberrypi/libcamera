.. SPDX-License-Identifier: CC-BY-SA-4.0

.. include:: documentation-contents.rst

************
Introduction
************

.. toctree::
   :hidden:

   API <api-html/index>
   Internal API <internal-api-html/index>

What is libcamera?
==================

libcamera is an open source complex camera support library for Linux, Android
and ChromeOS. The library interfaces with Linux kernel device drivers and
provides an intuitive API to developers in order to simplify the complexity
involved in capturing images from complex cameras on Linux systems.

What is a "complex camera"?
===========================

A modern "camera" tends to infact be several different pieces of hardware which
must all be controlled together in order to produce and capture images of
appropriate quality. A hardware pipeline typically consists of a camera sensor
that captures raw frames and transmits them on a bus, a receiver that decodes
the bus signals, and an image signal processor that processes raw frames to
produce usable images in a standard format. The Linux kernel handles these
multimedia devices through the 'Linux media' subsystem and provides a set of
application programming interfaces known collectively as the
V4L2 (`Video for Linux 2`_) and the `Media Controller`_ APIs, which provide an
interface to interact and control media devices.

.. _Video for Linux 2: https://www.linuxtv.org/downloads/v4l-dvb-apis-new/userspace-api/v4l/v4l2.html
.. _Media Controller: https://www.linuxtv.org/downloads/v4l-dvb-apis-new/userspace-api/mediactl/media-controller.html

Included in this subsystem are drivers for camera sensors, CSI2 (Camera
Serial Interface) receivers, and ISPs (Image Signal Processors).

The usage of these drivers to provide a functioning camera stack is a
responsibility that lies in userspace, and is commonly implemented separately
by vendors without a common architecture or API for application developers. This
adds a lot of complexity to the task, particularly when considering that the
differences in hardware pipelines and their representation in the kernel's APIs
often necessitate bespoke handling.

What is libcamera for?
======================

libcamera provides a complete camera stack for Linux-based systems to abstract
the configuration of hardware and image control algorithms required to obtain
desirable results from the camera through the kernel's APIs, reducing those
operations to a simple and consistent method for developers. In short instead of
having to deal with this:

.. graphviz:: mali-c55.dot

you can instead simply deal with:

.. code-block:: python

  >>> import libcamera as lc
  >>> camera_manager = lc.CameraManager.singleton()
  [0:15:59.582029920] [504]  INFO Camera camera_manager.cpp:313 libcamera v0.3.0+182-01e57380
  >>> for camera in camera_manager.cameras:
  ...     print(f' - {camera.id}')
  ...
   - mali-c55 tpg
   - imx415 1-001a

The library handles the rest for you. These documentary pages give more
information on the internal workings of libcamera (and the kernel camera stack
that lies behind it) as well as guidance on using libcamera in an application or
extending the library with support for your hardware (through the pipeline
handler and IPA module writer's guides).

How should I use it?
====================

There are a few ways you might want to use libcamera, depending on your
application. It's always possible to use the library directly, and you can find
detailed information on how to do so in the
:doc:`application writer's guide <guides/application-developer>`.

It is often more appropriate to use one of the frameworks with libcamera
support. For example an application powering an embedded media device
incorporating capture, encoding and streaming of both video and audio would
benefit from using `GStreamer`_, for which libcamera provides a plugin.
Similarly an application for user-facing devices like a laptop would likely
benefit accessing cameras through the XDG camera portal and `pipewire`_, which
brings the advantages of resource sharing (multiple applications accessing the
stream at the same time) and access control.

.. _GStreamer: https://gstreamer.freedesktop.org/
.. _pipewire: https://pipewire.org/

Camera Stack
============

::

    a c /    +-------------+  +-------------+  +-------------+  +-------------+
    p a |    |   Native    |  |  Framework  |  |   Native    |  |   Android   |
    p t |    |    V4L2     |  | Application |  |  libcamera  |  |   Camera    |
    l i |    | Application |  | (gstreamer) |  | Application |  |  Framework  |
    i o \    +-------------+  +-------------+  +-------------+  +-------------+
      n             ^                ^                ^                ^
                    |                |                |                |
    l a             |                |                |                |
    i d             v                v                |                v
    b a /    +-------------+  +-------------+         |         +-------------+
    c p |    |    V4L2     |  |   Camera    |         |         |   Android   |
    a t |    |   Compat.   |  |  Framework  |         |         |   Camera    |
    m a |    |             |  | (gstreamer) |         |         |     HAL     |
    e t \    +-------------+  +-------------+         |         +-------------+
    r i             ^                ^                |                ^
    a o             |                |                |                |
      n             |                |                |                |
        /           |         ,................................................
        |           |         !      :            Language             :      !
    l f |           |         !      :            Bindings             :      !
    i r |           |         !      :           (optional)            :      !
    b a |           |         \...............................................'
    c m |           |                |                |                |
    a e |           |                |                |                |
    m w |           v                v                v                v
    e o |    +----------------------------------------------------------------+
    r r |    |                                                                |
    a k |    |                           libcamera                            |
        |    |                                                                |
        \    +----------------------------------------------------------------+
                            ^                  ^                  ^
    Userspace               |                  |                  |
   ------------------------ | ---------------- | ---------------- | ---------------
    Kernel                  |                  |                  |
                            v                  v                  v
                      +-----------+      +-----------+      +-----------+
                      |   Media   | <--> |   Video   | <--> |   V4L2    |
                      |  Device   |      |  Device   |      |  Subdev   |
                      +-----------+      +-----------+      +-----------+

The camera stack comprises four software layers. From bottom to top:

* The kernel drivers control the camera hardware and expose a
  low-level interface to userspace through the Linux kernel V4L2
  family of APIs (Media Controller API, V4L2 Video Device API and
  V4L2 Subdev API).

* The libcamera framework is the core part of the stack. It
  handles all control of the camera devices in its core component,
  libcamera, and exposes a native C++ API to upper layers. Optional
  language bindings allow interfacing to libcamera from other
  programming languages.

  Those components live in the same source code repository and
  all together constitute the libcamera framework.

* The libcamera adaptation is an umbrella term designating the
  components that interface to libcamera in other frameworks.
  Notable examples are a V4L2 compatibility layer, a gstreamer
  libcamera element, and an Android camera HAL implementation based
  on libcamera.

  Those components can live in the libcamera project source code
  in separate repositories, or move to their respective project's
  repository (for instance the gstreamer libcamera element).

* The applications and upper level frameworks are based on the
  libcamera framework or libcamera adaptation, and are outside of
  the scope of the libcamera project.

V4L2 Compatibility Layer
  V4L2 compatibility is achieved through a shared library that traps all
  accesses to camera devices and routes them to libcamera to emulate high-level
  V4L2 camera devices. It is injected in a process address space through
  ``LD_PRELOAD`` and is completely transparent for applications.

  The compatibility layer exposes camera device features on a best-effort basis,
  and aims for the level of features traditionally available from a UVC camera
  designed for video conferencing.

Android Camera HAL
  Camera support for Android is achieved through a generic Android camera HAL
  implementation on top of libcamera. The HAL implements features required by
  Android and out of scope from libcamera, such as JPEG encoding support.

  This component is used to provide support for ChromeOS platforms.

GStreamer element (gstlibcamerasrc)
  A `GStreamer element`_ is provided to allow capture from libcamera supported
  devices through GStreamer pipelines, and connect to other elements for further
  processing.

Native libcamera API
  Applications can make use of the libcamera API directly using the C++
  API. An example application and walkthrough using the libcamera API can be
  followed in the :doc:`Application writer's guide </guides/application-developer>`

.. _GStreamer element: https://gstreamer.freedesktop.org/documentation/application-development/basics/elements.html

Licensing
=========

The libcamera core is covered by the `LGPL-2.1-or-later`_ license. Pipeline
Handlers are a part of the libcamera code base and need to be contributed
upstream by device vendors. IPA modules included in libcamera are covered by a
free software license, however third-parties may develop IPA modules outside of
libcamera and distribute them under a closed-source license, provided they do
not include source code from the libcamera project.

The libcamera project itself contains multiple libraries, applications and
utilities. Licenses are expressed through SPDX tags in text-based files that
support comments, and through the .reuse/dep5 file otherwise. A copy of all
licenses are stored in the LICENSES directory, and a full summary of the
licensing used throughout the project can be found in the COPYING.rst document.

Applications which link dynamically against libcamera and use only the public
API are an independent work of the authors and have no license restrictions
imposed upon them from libcamera.

.. _LGPL-2.1-or-later: https://spdx.org/licenses/LGPL-2.1-or-later.html
