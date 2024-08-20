.. SPDX-License-Identifier: CC-BY-SA-4.0

.. contents::
   :local:

.. include:: documentation-contents.rst

*************
Documentation
*************

.. toctree::
   :hidden:

   API <api-html/index>

API
===

The libcamera API is extensively documented using Doxygen. The :ref:`API
nightly build <api>` contains the most up-to-date API documentation, built from
the latest master branch.

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
